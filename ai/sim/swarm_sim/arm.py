"""Adaptador de brazo simulado (implementa IRobotArm sobre PyBullet).

Control: el TCP sigue una recta (waypoint) a la velocidad pedida; en cada paso se resuelve
IK y se envian objetivos articulares a motores en POSITION_CONTROL (dinamica real: gravedad,
limites de velocidad/par del URDF). Cada maniobra es un generador que avanza un paso de
simulacion por `yield`; asi varios brazos se ejecutan a la vez sin hilos.

LIMITACION CONOCIDA: el agarre fija el objeto al TCP con una restriccion rigida (tras cerrar
la pinza y verificar distancia/peso/anchura); no modela friccion de la pinza real.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np
import pybullet as p

from .types import (TERMINAL_STATUSES, ExecutionStatus, ManeuverCommand, ManeuverResult,
                    ManeuverType, Pose, RobotCapabilities, RobotState)


class ManeuverFailure(Exception):
    def __init__(self, code: str, detail: str = ""):
        super().__init__(code)
        self.code, self.detail = code, detail


@dataclass
class _Execution:
    cmd: ManeuverCommand
    status: ExecutionStatus
    t_submit: float
    detail: str = ""
    t_start: Optional[float] = None
    t_end: Optional[float] = None
    error_m: Optional[float] = None
    data: dict = field(default_factory=dict)
    gen: Any = None
    goal: Optional[np.ndarray] = None  # posicion objetivo del TCP (para prediccion de colisiones)


class SimRobotArm:
    def __init__(self, world, robot_id: str, model: dict, base_pos, base_yaw: float):
        self.world, self.bc = world, world.bc
        self.id, self.cfg, self.dt = robot_id, model, world.dt
        self.base_pos = np.array(base_pos, dtype=float)
        self.base_yaw = base_yaw
        bc = self.bc
        self.body = bc.loadURDF(model["urdf"], list(base_pos),
                                p.getQuaternionFromEuler([0, 0, base_yaw]), useFixedBase=True,
                                globalScaling=float(model.get("urdf_global_scaling", 1.0)))
        self.ee = model["ee_link"]
        self.arm_joints = list(model["arm_joints"])
        self.finger_joints = list(model["finger_joints"])

        movable = [j for j in range(bc.getNumJoints(self.body))
                   if bc.getJointInfo(self.body, j)[2] != p.JOINT_FIXED]
        info = {j: bc.getJointInfo(self.body, j) for j in movable}
        self._movable = movable
        self._ll = [info[j][8] for j in movable]
        self._ul = [info[j][9] for j in movable]
        self._jr = [u - l for l, u in zip(self._ll, self._ul)]
        self._rest = [model["home_joints"][self.arm_joints.index(j)] if j in self.arm_joints
                      else model["gripper_open_m"] for j in movable]
        self.joint_limits = {j: (info[j][8], info[j][9]) for j in self.arm_joints}
        self._max_force = [info[j][10] for j in self.arm_joints]
        self._max_vel = [info[j][11] for j in self.arm_joints]
        self._near_limit: set = set()

        self.tool_down = tuple(p.getQuaternionFromEuler([math.pi, 0.0, base_yaw]))
        self.executions: dict = {}
        self.active: Optional[_Execution] = None
        self.moving = False
        self.held: Optional[str] = None
        self._constraint = None
        self.comm_lost_until = -1.0
        self._last_err: Optional[float] = None
        # 1 = comportamiento original. 2/4 = mantener el ultimo objetivo
        # articular entre actualizaciones IK; fisica y seguridad siguen a 240 Hz.
        self._ik_stride = int(os.environ.get("SWARM_IK_STRIDE", "1"))
        if self._ik_stride < 1 or self._ik_stride > 8:
            raise ValueError("SWARM_IK_STRIDE debe estar entre 1 y 8")
        self._ik_skipped = 0
        self._go_home()

    # ------------------------------------------------------------ cinematica
    def tcp(self):
        ls = self.bc.getLinkState(self.body, self.ee, computeForwardKinematics=True)
        return np.array(ls[4]), tuple(ls[5])

    def _ik(self, pos, orn):
        q = self.bc.calculateInverseKinematics(
            self.body, self.ee, [float(v) for v in pos], list(orn),
            lowerLimits=self._ll, upperLimits=self._ul, jointRanges=self._jr,
            restPoses=self._rest, maxNumIterations=60, residualThreshold=1e-4)
        return list(q[:len(self.arm_joints)])

    def _servo(self, pos, orn):
        self._drive(self._ik(pos, orn))

    def _drive(self, q):
        for k, j in enumerate(self.arm_joints):
            self.bc.setJointMotorControl2(self.body, j, p.POSITION_CONTROL, targetPosition=q[k],
                                          force=self._max_force[k], maxVelocity=self._max_vel[k])

    def _go_home(self):
        bc, m = self.bc, self.cfg
        for j, q in zip(self.arm_joints, m["home_joints"]):
            bc.resetJointState(self.body, j, q)
        for j in self.finger_joints:
            bc.resetJointState(self.body, j, m["gripper_open_m"])
        c, s = math.cos(self.base_yaw), math.sin(self.base_yaw)
        f, h = m["home_tcp_forward_m"], m["home_tcp_height_m"]
        home = self.base_pos + np.array([c * f, s * f, h])
        for _ in range(80):  # converge IK sobre el estado real
            q = self._ik(home, self.tool_down)
            for j, qq in zip(self.arm_joints, q):
                bc.resetJointState(self.body, j, qq)
        self._drive(q)
        self._gripper_targets(m["gripper_open_m"])
        self.wp = self.tcp()[0]

    # ------------------------------------------------------------- contrato
    def capabilities(self) -> RobotCapabilities:
        m = self.cfg
        return RobotCapabilities(self.id, [ManeuverType(x) for x in m["maneuvers"]], m["reach_m"],
                                 m["payload_kg"], m["max_tcp_speed_mps"], 2 * m["gripper_open_m"])

    def state(self) -> RobotState:
        w, bc = self.world, self.bc
        lost = w.time < self.comm_lost_until
        pos, orn = self.tcp()
        js = bc.getJointStates(self.body, self.arm_joints)
        fingers = bc.getJointStates(self.body, self.finger_joints)
        status = "comm_lost" if lost else ("running" if self.active else "idle")
        return RobotState(
            id=self.id, sim_time=w.time, valid=not lost, available=self.active is None and not lost,
            status=status, tcp=Pose(tuple(pos), orn), joint_positions=[s[0] for s in js],
            joint_velocities=[s[1] for s in js], gripper_opening_m=sum(f[0] for f in fingers),
            held_object=self.held or "", active_execution=self.active.cmd.execution_id if self.active else "")

    def result(self, execution_id: str) -> ManeuverResult:
        ex = self.executions[execution_id]
        end = ex.t_end if ex.t_end is not None else self.world.time
        return ManeuverResult(execution_id, ex.cmd.task_id, self.id, ex.status, ex.detail, self.world.time,
                              0.0 if ex.t_start is None else end - ex.t_start, ex.error_m, dict(ex.data))

    def submit(self, cmd: ManeuverCommand) -> ManeuverResult:
        w = self.world
        if cmd.execution_id in self.executions:
            return ManeuverResult(cmd.execution_id, cmd.task_id, self.id, ExecutionStatus.REJECTED,
                                  "duplicate_execution_id", w.time)
        ex = _Execution(cmd, ExecutionStatus.ACCEPTED, w.time)
        self.executions[cmd.execution_id] = ex
        w.telemetry.event(w.time, "submitted", self.id, cmd.execution_id, maneuver=cmd.type.value, task_id=cmd.task_id)
        reason = self._validate(cmd, ex)
        if reason:
            ex.status, ex.detail, ex.t_end = ExecutionStatus.REJECTED, reason, w.time
            w.telemetry.event(w.time, "rejected", self.id, cmd.execution_id, reason=reason)
            return self.result(cmd.execution_id)
        ex.gen = self._make_gen(cmd)
        self.active = ex
        w.telemetry.event(w.time, "accepted", self.id, cmd.execution_id)
        return self.result(cmd.execution_id)

    def execute(self, cmd: ManeuverCommand) -> ManeuverResult:
        """Bloqueante (semantica de C++ v0.2): avanza TODO el mundo hasta que termina esta maniobra."""
        res = self.submit(cmd)
        if res.status == ExecutionStatus.REJECTED:
            return res
        ex = self.executions[cmd.execution_id]
        limit = (cmd.timeout_s or self.world.safety["default_timeout_s"]) + 2.0
        self.world.run_until(lambda: ex.status in TERMINAL_STATUSES, limit)
        return self.result(cmd.execution_id)

    def cancel(self, id: str) -> bool:
        ex = self.active
        if ex and id in (ex.cmd.execution_id, ex.cmd.task_id):
            self._finish(ex, ExecutionStatus.CANCELLED, "cancelled_by_core")
            return True
        return False

    def abort(self, code: str, detail: str = "") -> None:
        if self.active:
            self._finish(self.active, ExecutionStatus.FAILED, f"{code}: {detail}".strip(": "))

    # ----------------------------------------------------------- validacion
    def _goal_position(self, cmd: ManeuverCommand):
        t = cmd.type
        if t in (ManeuverType.MOVE, ManeuverType.SORT):
            return np.array(cmd.target.position, dtype=float)
        if t in (ManeuverType.GRASP, ManeuverType.INSPECT):
            return self.world.objects[cmd.object_id].position()
        return None

    def _validate(self, cmd: ManeuverCommand, ex: _Execution) -> Optional[str]:
        w, m, sf = self.world, self.cfg, self.world.safety
        if w.time < self.comm_lost_until:
            return "comm_lost"
        if self.active is not None:
            return f"busy: {self.active.cmd.execution_id}"
        if cmd.type.value not in m["maneuvers"]:
            return "unsupported_maneuver"
        if not (0.0 < cmd.max_speed <= m["max_tcp_speed_mps"]) or cmd.tolerance <= 0:
            return "invalid_speed_or_tolerance"
        if cmd.type in (ManeuverType.MOVE, ManeuverType.SORT):
            if cmd.target is None:
                return "missing_target"
            if cmd.target.frame != "world":
                return f"unsupported_frame: {cmd.target.frame}"
        if cmd.type in (ManeuverType.GRASP, ManeuverType.INSPECT):
            if cmd.object_id not in w.objects:
                return f"unknown_object: {cmd.object_id}"
        if cmd.type == ManeuverType.GRASP:
            obj = w.objects[cmd.object_id]
            if self.held:
                return "gripper_occupied"
            if obj.held_by:
                return f"object_held_by_other: {obj.held_by}"
            if obj.mass > m["payload_kg"]:
                return f"payload_exceeded: {obj.mass} kg > {m['payload_kg']} kg"
            if obj.grip_width > 2 * m["gripper_open_m"]:
                return "object_too_wide_for_gripper"
        if cmd.type in (ManeuverType.SORT, ManeuverType.RELEASE) and not self.held:
            return "not_holding_object"
        if cmd.type == ManeuverType.SORT and self.held and cmd.object_id and cmd.object_id != self.held:
            return f"holding_other_object: {self.held}"

        goal = self._goal_position(cmd)
        ex.goal = goal
        if goal is not None:
            if goal[2] < sf["min_tcp_z_m"]:
                return "below_floor"
            shoulder = self.base_pos + np.array([0, 0, m["shoulder_height_m"]])
            if np.linalg.norm(goal - shoulder) > m["reach_m"]:
                return "out_of_reach"
            for other in w.arms.values():  # colision PREVISTA contra posicion actual u objetivo de otros brazos
                if other is self:
                    continue
                refs = [other.tcp()[0]]
                if other.active is not None and other.active.goal is not None:
                    refs.append(other.active.goal)
                if any(np.linalg.norm(goal - r) < sf["min_tcp_separation_m"] for r in refs):
                    return f"predicted_collision_with: {other.id}"
        return None

    # ----------------------------------------------------------- ejecucion
    def update(self) -> None:
        """Un paso de control (se llama antes de cada paso de fisica)."""
        w = self.world
        self._check_limits()
        ex = self.active
        if ex is None:
            return
        if ex.status == ExecutionStatus.ACCEPTED:
            ex.status, ex.t_start = ExecutionStatus.RUNNING, w.time
            w.telemetry.event(w.time, "started", self.id, ex.cmd.execution_id)
        limit = ex.cmd.timeout_s or w.safety["default_timeout_s"]
        if w.time - ex.t_start > limit:
            self._finish(ex, ExecutionStatus.FAILED, "timeout")
            return
        try:
            next(ex.gen)
        except StopIteration:
            self._finish(ex, ExecutionStatus.COMPLETED, "ok")
        except ManeuverFailure as f:
            self._finish(ex, ExecutionStatus.FAILED, f"{f.code}: {f.detail}".strip(": "))

    def _finish(self, ex: _Execution, status: ExecutionStatus, detail: str) -> None:
        w = self.world
        if ex.gen is not None:
            ex.gen.close()
        ex.status, ex.detail, ex.t_end, ex.error_m = status, detail, w.time, self._last_err
        self.active, self.moving = None, False
        if status != ExecutionStatus.COMPLETED:  # congelar donde esta
            self.wp = self.tcp()[0]
            self._drive([s[0] for s in self.bc.getJointStates(self.body, self.arm_joints)])
        w.telemetry.event(w.time, status.value, self.id, ex.cmd.execution_id, detail=detail, error_m=ex.error_m)

    def _check_limits(self) -> None:
        w = self.world
        margin = w.safety["limit_margin_rad"]
        for j, s in zip(self.arm_joints, self.bc.getJointStates(self.body, self.arm_joints)):
            lo, hi = self.joint_limits[j]
            near = s[0] < lo + margin or s[0] > hi - margin
            if near and j not in self._near_limit:
                self._near_limit.add(j)
                w.telemetry.event(w.time, "joint_limit", self.id, self.active.cmd.execution_id if self.active else None,
                                  joint=j, position=s[0])
            elif not near:
                self._near_limit.discard(j)

    # ---------------------------------------------------------- primitivas
    def _move_to(self, goal, orn, speed, tol):
        goal = np.array(goal, dtype=float)
        self.moving = True
        ik_tick = 0
        settle_tick = 0
        try:
            while True:
                delta = goal - self.wp
                dist = float(np.linalg.norm(delta))
                if dist < 1e-9:
                    break
                self.wp = self.wp + delta / dist * min(speed * self.dt, dist)
                # En subpasos intermedios, Bullet mantiene el ultimo objetivo
                # POSITION_CONTROL. Resolver siempre el primer y ultimo waypoint.
                if ik_tick % self._ik_stride == 0 or dist <= speed * self.dt:
                    self._servo(self.wp, orn)
                else:
                    self._ik_skipped += 1
                ik_tick += 1
                yield
            t0 = self.world.time  # asentamiento: se exige llegar de verdad
            while True:
                self._last_err = float(np.linalg.norm(goal - self.tcp()[0]))
                if self._last_err <= tol:
                    return
                if self.world.time - t0 > 2.0:
                    raise ManeuverFailure("target_not_reached", f"error={self._last_err:.4f} m > tol={tol} m")
                # Durante asentamiento, la consigna cartesiana es constante.
                # Revalidar periodicamente para mantener el lazo de control.
                if settle_tick % self._ik_stride == 0:
                    self._servo(goal, orn)
                else:
                    self._ik_skipped += 1
                settle_tick += 1
                yield
        finally:
            self.moving = False

    def _gripper_targets(self, opening):
        for j in self.finger_joints:
            self.bc.setJointMotorControl2(self.body, j, p.POSITION_CONTROL, targetPosition=opening,
                                          force=self.cfg["gripper_force_n"])

    def _gripper(self, opening, timeout=0.8):
        self._gripper_targets(opening)
        t0 = self.world.time
        while True:
            yield
            pos = [s[0] for s in self.bc.getJointStates(self.body, self.finger_joints)]
            if all(abs(x - opening) < 0.003 for x in pos) or self.world.time - t0 > timeout:
                return

    def _attach(self, obj):
        bc = self.bc
        ls = bc.getLinkState(self.body, self.ee, computeForwardKinematics=True)
        inv_p, inv_o = bc.invertTransform(ls[0], ls[1])  # frame del centro de masas del enlace
        opos, oorn = bc.getBasePositionAndOrientation(obj.body)
        rel_p, rel_o = bc.multiplyTransforms(inv_p, inv_o, opos, oorn)
        self._constraint = bc.createConstraint(self.body, self.ee, obj.body, -1, p.JOINT_FIXED,
                                               [0, 0, 0], rel_p, [0, 0, 0], rel_o, [0, 0, 0, 1])
        bc.changeConstraint(self._constraint, maxForce=300)
        for link in range(-1, bc.getNumJoints(self.body)):
            bc.setCollisionFilterPair(self.body, obj.body, link, -1, 0)
        obj.held_by, self.held = self.id, obj.id
        self.world.telemetry.event(self.world.time, "grasped", self.id, self.active.cmd.execution_id, object_id=obj.id)

    def _detach(self):
        obj = self.world.objects[self.held]
        self.bc.removeConstraint(self._constraint)
        for link in range(-1, self.bc.getNumJoints(self.body)):
            self.bc.setCollisionFilterPair(self.body, obj.body, link, -1, 1)
        self.world.telemetry.event(self.world.time, "released", self.id,
                                   self.active.cmd.execution_id if self.active else None, object_id=obj.id)
        obj.held_by, self.held, self._constraint = "", None, None

    def _wait(self, seconds):
        n = max(1, int(round(seconds / self.dt)))
        for _ in range(n):
            yield

    # ------------------------------------------------------------ maniobras
    def _make_gen(self, cmd: ManeuverCommand):
        return {ManeuverType.MOVE: self._run_move, ManeuverType.GRASP: self._run_grasp,
                ManeuverType.INSPECT: self._run_inspect, ManeuverType.HOLD: self._run_hold,
                ManeuverType.RELEASE: self._run_release, ManeuverType.SORT: self._run_sort}[cmd.type](cmd)

    def _orn(self, cmd):
        return cmd.target.orientation if (cmd.target and cmd.target.orientation) else self.tool_down

    def _run_move(self, cmd):
        yield from self._move_to(cmd.target.position, self._orn(cmd), cmd.max_speed, cmd.tolerance)

    def _run_inspect(self, cmd):
        obj = self.world.objects[cmd.object_id]
        above = obj.position() + np.array([0, 0, 0.25])
        yield from self._move_to(above, self.tool_down, cmd.max_speed, cmd.tolerance)
        o = self.world.env.observe_object(obj)
        self.executions[cmd.execution_id].data["observation"] = {
            "object_id": o.object_id, "position": list(o.pose.position), "zone": o.zone, "held_by": o.held_by}

    def _run_grasp(self, cmd):
        m, obj = self.cfg, self.world.objects[cmd.object_id]
        planned = obj.position()  # ultima observacion; el agarre es en lazo abierto
        hover = planned + np.array([0, 0, m["approach_clearance_m"]])
        grasp = planned + np.array([0, 0, m["tcp_grasp_offset_m"]])
        yield from self._gripper(m["gripper_open_m"])
        yield from self._move_to(hover, self.tool_down, cmd.max_speed, cmd.tolerance)
        yield from self._move_to(grasp, self.tool_down, min(cmd.max_speed, 0.2), cmd.tolerance)
        d = float(np.linalg.norm(self.tcp()[0] - obj.position()))
        if d > 0.03 + m["tcp_grasp_offset_m"]:
            raise ManeuverFailure("grasp_failed", f"objeto a {d:.3f} m del TCP")
        yield from self._gripper(max(0.0, obj.grip_width / 2 - 0.002))
        self._attach(obj)
        yield from self._move_to(hover, self.tool_down, cmd.max_speed, cmd.tolerance)

    def _run_hold(self, cmd):
        pos, orn = self.wp.copy(), self.tcp()[1]
        steps = None if cmd.hold_time is None else max(1, int(round(cmd.hold_time / self.dt)))
        i = 0
        while steps is None or i < steps:
            self._servo(pos, orn)
            i += 1
            yield

    def _run_release(self, cmd):
        self._detach()
        yield from self._gripper(self.cfg["gripper_open_m"])
        yield from self._wait(0.3)

    def _run_sort(self, cmd):
        m, tgt = self.cfg, np.array(cmd.target.position, dtype=float)
        hover = tgt + np.array([0, 0, m["approach_clearance_m"]])
        yield from self._move_to(hover, self.tool_down, cmd.max_speed, cmd.tolerance)
        yield from self._move_to(tgt, self.tool_down, min(cmd.max_speed, 0.2), cmd.tolerance)
        self._detach()
        yield from self._gripper(m["gripper_open_m"])
        yield from self._wait(0.3)
        yield from self._move_to(hover, self.tool_down, cmd.max_speed, cmd.tolerance)

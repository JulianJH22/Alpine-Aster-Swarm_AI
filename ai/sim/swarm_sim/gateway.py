"""Pasarela TCP + JSON-lines entre el nucleo (C++) y el simulador (Python).

Una peticion por linea:   {"id": 1, "op": "submit", "args": {...}}
Una respuesta por linea:  {"id": 1, "ok": true, "result": {...}}  |  {"id": 1, "ok": false, "error": "..."}

El TIEMPO LO CONTROLA EL CLIENTE (op "step" / "run_until_idle"): el simulador no avanza solo.
Asi las pruebas son reproducibles y el MPC puede sincronizarse con el reloj de simulacion.
Atiende a un cliente a la vez (suficiente para el ordenador central).
"""
from __future__ import annotations

import json
import socket
import socketserver
from typing import Any

from .types import ManeuverCommand, to_plain
from .world import SimWorld

PROTOCOL = "swarm-sim/0.1"


class Gateway:
    def __init__(self, world: SimWorld):
        self.world = world

    def handle(self, req: dict) -> dict:
        rid = req.get("id")
        try:
            op, args = req["op"], req.get("args", {})
            fn = getattr(self, f"op_{op}", None)
            if fn is None:
                raise ValueError(f"op desconocida: {op}")
            return {"id": rid, "ok": True, "result": to_plain(fn(**args))}
        except Exception as e:  # el error vuelve al cliente; nunca tumba el servidor
            return {"id": rid, "ok": False, "error": f"{type(e).__name__}: {e}"}

    # --- operaciones -------------------------------------------------------
    def op_hello(self) -> dict:
        w = self.world
        return {"protocol": PROTOCOL, "scenario": w.cfg.get("scenario_name"), "dt": w.dt,
                "sim_time": w.time, "robots": list(w.arms), "objects": list(w.objects),
                "units": {"length": "m", "angle": "rad", "time": "s"}, "orientation": "quaternion xyzw",
                "frames": ["world"]}

    def op_time(self) -> dict:
        return {"sim_time": self.world.time, "step_index": self.world.step_index}

    def op_states(self) -> list:
        return [a.state() for a in self.world.arms.values()]

    def op_state(self, robot_id: str):
        return self.world.arms[robot_id].state()

    def op_capabilities(self, robot_id: str):
        return self.world.arms[robot_id].capabilities()

    def op_observe(self):
        return self.world.env.observe()

    # FASE 5.3: consulta predictiva y envio condicionado; no entrena MAPPO.
    def op_plan_move(self, robot_id: str, target: list, max_speed: float = 0.1,
                     horizon_steps: int = 12) -> dict:
        from .mpc_preflight import plan_tcp
        import numpy as np
        w = self.world
        arm = w.arms[robot_id]
        shoulder = arm.base_pos + np.array([0, 0, arm.cfg["shoulder_height_m"]])
        others = [a.tcp()[0] for a in w.arms.values() if a is not arm]
        result = plan_tcp(arm.tcp()[0], target, shoulder, arm.cfg["reach_m"],
                          others, w.safety["min_tcp_separation_m"],
                          w.safety["min_tcp_z_m"], max_speed, w.dt, horizon_steps)
        if w.platform_moving:
            result["feasible_preflight"] = False
            result["reason"] = "platform_moving"
        elif arm.active is not None:
            result["feasible_preflight"] = False
            result["reason"] = "robot_busy"
        elif any(a.active is not None for a in w.arms.values() if a is not arm):
            result["feasible_preflight"] = False
            result["reason"] = "other_robot_busy"
        result["robot_id"] = robot_id
        result["sim_time"] = w.time
        return result

    def op_submit_planned(self, command: dict, horizon_steps: int = 12) -> dict:
        """Preflight + submit atomicos respecto a este gateway secuencial."""
        from .types import ManeuverType
        cmd = ManeuverCommand.from_dict(command)
        if cmd.type != ManeuverType.MOVE or cmd.target is None:
            raise ValueError("submit_planned solo admite Move con target")
        if cmd.target.frame != "world":
            raise ValueError("solo se admite frame world")
        preview = self.op_plan_move(cmd.robot_id, list(cmd.target.position),
                                    cmd.max_speed, horizon_steps)
        if not preview["feasible_preflight"]:
            return {"submitted": False, "plan": preview}
        result = self.world.arms[cmd.robot_id].submit(cmd)
        return {"submitted": True, "plan": preview, "execution": to_plain(result)}

    def op_submit(self, command: dict):
        cmd = ManeuverCommand.from_dict(command)
        if cmd.robot_id not in self.world.arms:
            raise KeyError(f"robot desconocido: {cmd.robot_id}")
        return self.world.arms[cmd.robot_id].submit(cmd)

    def op_status(self, robot_id: str, execution_id: str):
        return self.world.arms[robot_id].result(execution_id)

    def op_cancel(self, robot_id: str, id: str) -> dict:
        return {"cancelled": self.world.arms[robot_id].cancel(id)}

    def op_step(self, seconds: float) -> dict:
        self.world.run_for(seconds)
        return {"sim_time": self.world.time}

    def op_run_until_idle(self, timeout_s: float = 120.0) -> dict:
        idle = self.world.run_until_idle(timeout_s)
        return {"idle": idle, "sim_time": self.world.time}

    def op_reset(self) -> dict:
        """Rebuild from the original scenario; invalidates previous body references and commands."""
        self.world.reset()
        return {"sim_time": self.world.time, "step_index": self.world.step_index,
                "robots": list(self.world.arms), "objects": list(self.world.objects)}

    def op_snapshot(self) -> dict:
        """Consistent read of measured state at the current simulation step."""
        w = self.world
        return {"sim_time": w.time, "step_index": w.step_index,
                "states": [a.state() for a in w.arms.values()],
                "observation": w.env.observe(),
                "platform_angle_rad": w.platform_angle,
                "platform_moving": w.platform_moving,
                "telemetry_samples": len(w.telemetry.samples),
                "telemetry_events": len(w.telemetry.events)}

    def op_telemetry_samples(self, since_index: int = 0, limit: int = 100) -> dict:
        if not isinstance(since_index, int) or since_index < 0:
            raise ValueError("since_index must be a nonnegative integer")
        if not isinstance(limit, int) or not 1 <= limit <= 1000:
            raise ValueError("limit must be between 1 and 1000")
        samples = self.world.telemetry.samples
        return {"samples": samples[since_index:since_index + limit],
                "next_index": min(len(samples), since_index + limit),
                "total": len(samples)}

    def op_telemetry(self, since_seq: int = 0) -> list:
        return [e for e in self.world.telemetry.events if e["seq"] >= since_seq]

    # FASE 5.2: operaciones lógicas de desmontaje y clasificación, sin movimientos de brazos.
    def op_recycling_inspect(self, component_id: str) -> dict:
        return self.world.recycling.inspect(component_id)

    def op_recycling_test(self, component_id: str) -> dict:
        return self.world.recycling.test_function(component_id)

    def op_recycling_disassemble(self, component_id: str, tool: str) -> dict:
        return self.world.recycling.disassemble(component_id, tool)

    def op_recycling_classify(self, component_id: str, material: str, functional: str) -> dict:
        return self.world.recycling.classify(component_id, material, functional)

    def op_recycling_summary(self) -> dict:
        return self.world.recycling.summary()

    def op_inject_comm_loss(self, robot_id: str, duration_s: float) -> dict:
        self.world.inject_comm_loss(robot_id, duration_s)
        return {"ok": True}


def make_server(world: SimWorld, host: str = "127.0.0.1", port: int = 7777) -> socketserver.TCPServer:
    gw = Gateway(world)

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            for line in self.rfile:
                line = line.strip()
                if not line:
                    continue
                try:
                    resp = gw.handle(json.loads(line))
                except json.JSONDecodeError as e:
                    resp = {"id": None, "ok": False, "error": f"JSON invalido: {e}"}
                self.wfile.write((json.dumps(resp) + "\n").encode())

    socketserver.TCPServer.allow_reuse_address = True
    return socketserver.TCPServer((host, port), Handler)


class GatewayClient:
    """Cliente minimo (Python) que sirve de referencia para el adaptador C++."""

    def __init__(self, host: str = "127.0.0.1", port: int = 7777):
        self.sock = socket.create_connection((host, port))
        self.f = self.sock.makefile("rwb")
        self._id = 0

    def call(self, op: str, **args) -> Any:
        self._id += 1
        self.f.write((json.dumps({"id": self._id, "op": op, "args": args}) + "\n").encode())
        self.f.flush()
        resp = json.loads(self.f.readline())
        if not resp["ok"]:
            raise RuntimeError(resp["error"])
        return resp["result"]

    def close(self) -> None:
        self.f.close()
        self.sock.close()

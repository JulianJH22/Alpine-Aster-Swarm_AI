"""Pruebas de integracion: escenarios (a)-(f) de la seccion 7 y criterios de la seccion 10."""
import math
import threading
from pathlib import Path

import numpy as np
import pytest

from swarm_sim import (ExecutionStatus as S, IEnvironment, IRobotArm, ManeuverCommand, ManeuverType as T, Pose,
                       SimWorld)
from swarm_sim.gateway import Gateway, GatewayClient, make_server
from swarm_sim.types import euler_to_quat, quat_to_euler

CONFIG = Path(__file__).resolve().parents[1] / "config" / "scenario.json"


def cmd(eid, robot, typ, **kw):
    return ManeuverCommand(eid, eid, robot, typ, **kw)


def event_types(world, robot_id=None, eid=None):
    return [e["type"] for e in world.telemetry.events
            if (robot_id is None or e["robot_id"] == robot_id) and (eid is None or e["execution_id"] == eid)]


@pytest.fixture
def world():
    w = SimWorld(CONFIG)
    yield w
    w.close()


# ------------------------------------------------------------ criterios generales
def test_six_arms_identified_with_valid_state(world):
    assert list(world.arms) == [f"arm_{i}" for i in range(1, 7)]
    for rid, arm in world.arms.items():
        assert isinstance(arm, IRobotArm)
        s = arm.state()
        assert s.id == rid and s.valid and s.available and s.status == "idle"
        assert len(s.joint_positions) == 7 and s.sim_time == 0.0
    assert isinstance(world.env, IEnvironment)


def test_hexagon_geometry_comes_from_config(world):
    R = world.cfg["hexagon"]["circumradius_m"]
    for k, arm in enumerate(world.arms.values()):
        ang = k * math.pi / 3
        assert np.allclose(arm.base_pos[:2], [R * math.cos(ang), R * math.sin(ang)], atol=1e-9)


def test_units_and_quaternions():
    q = euler_to_quat(0.3, -0.2, 1.1)
    assert np.allclose(quat_to_euler(q), (0.3, -0.2, 1.1), atol=1e-9)
    legacy = Pose.from_dict({"x": 1, "y": 2, "z": 3, "roll": 0, "pitch": 0, "yaw": 0})  # Pose de C++ v0.2
    assert legacy.position == (1, 2, 3) and legacy.orientation is None


# ------------------------------------------------------------------ (a) mover
def test_a_free_move_lifecycle_and_accuracy(world):
    arm = world.arms["arm_1"]
    target = (0.15, 0.0, 0.25)
    res = arm.submit(cmd("a1", "arm_1", T.MOVE, target=Pose(target)))
    assert res.status == S.ACCEPTED
    world.run_for(0.1)
    assert arm.result("a1").status == S.RUNNING and arm.state().status == "running"
    world.run_until_idle(20)
    res = arm.result("a1")
    assert res.status == S.COMPLETED and res.error_m <= 0.01
    assert np.linalg.norm(arm.tcp()[0] - target) <= 0.01
    assert event_types(world, "arm_1", "a1") == ["submitted", "accepted", "started", "completed"]


# ------------------------------------------- (b) agarrar, trasladar, liberar / clasificar
def test_b_grasp_move_release(world):
    arm, obj = world.arms["arm_1"], world.objects["obj_1"]
    assert arm.execute(cmd("g", "arm_1", T.GRASP, object_id="obj_1")).status == S.COMPLETED
    assert obj.held_by == "arm_1" and arm.state().held_object == "obj_1"
    assert arm.execute(cmd("m", "arm_1", T.MOVE, target=Pose((0.22, 0.05, 0.04)))).status == S.COMPLETED
    assert np.linalg.norm(obj.position() - np.array([0.22, 0.05, 0.04])) < 0.03  # el objeto viaja con el TCP
    assert arm.execute(cmd("r", "arm_1", T.RELEASE)).status == S.COMPLETED
    world.run_for(0.5)
    o = next(x for x in world.env.observe().objects if x.object_id == "obj_1")
    assert o.held_by == "" and o.pose.position[2] < 0.05 and abs(o.pose.position[0] - 0.22) < 0.04


def test_b_two_arms_sort_simultaneously(world):
    a1, a4 = world.arms["arm_1"], world.arms["arm_4"]
    a1.submit(cmd("g1", "arm_1", T.GRASP, object_id="obj_1"))
    a4.submit(cmd("g4", "arm_4", T.GRASP, object_id="obj_2"))
    assert world.run_until_idle(30)
    a1.submit(cmd("s1", "arm_1", T.SORT, target=Pose((0.32, 0.0, 0.022))))
    a4.submit(cmd("s4", "arm_4", T.SORT, target=Pose((-0.32, 0.0, 0.022))))
    assert world.run_until_idle(30)
    world.run_for(0.5)
    assert all(a.result(e).status == S.COMPLETED for a, e in ((a1, "g1"), (a4, "g4"), (a1, "s1"), (a4, "s4")))
    zones = {o.object_id: o.zone for o in world.env.observe().objects}
    assert zones["obj_1"] == "zone_util" and zones["obj_2"] == "zone_scrap"
    assert not world.telemetry.events_of("collision")


def test_inspect_returns_observation(world):
    res = world.arms["arm_1"].execute(cmd("i", "arm_1", T.INSPECT, object_id="obj_1"))
    assert res.status == S.COMPLETED
    assert np.allclose(res.data["observation"]["position"][:2], [0.10, 0.0], atol=1e-3)


# ------------------------------------------------ (c) uno sujeta mientras otro actua
def test_c_hold_while_other_arm_moves(world):
    a1, a4 = world.arms["arm_1"], world.arms["arm_4"]
    assert a1.execute(cmd("g", "arm_1", T.GRASP, object_id="obj_1")).status == S.COMPLETED
    z0 = world.objects["obj_1"].position()[2]
    a1.submit(cmd("h", "arm_1", T.HOLD))  # indefinido hasta cancelar
    a4.submit(cmd("m4", "arm_4", T.MOVE, target=Pose((-0.1, 0.0, 0.25))))
    world.run_until(lambda: a4.active is None, 20)
    assert a4.result("m4").status == S.COMPLETED
    assert a1.result("h").status == S.RUNNING and a1.held == "obj_1"
    assert abs(world.objects["obj_1"].position()[2] - z0) < 0.01  # no se cae mientras sujeta
    assert a1.cancel("h") and a1.result("h").status == S.CANCELLED and a1.held == "obj_1"


# --------------------------- (d) misma zona: se bloquea la ejecucion insegura
def test_d_predicted_collision_is_rejected(world):
    same = Pose((0.1, 0.0, 0.2))
    assert world.arms["arm_1"].submit(cmd("d1", "arm_1", T.MOVE, target=same)).status == S.ACCEPTED
    r = world.arms["arm_4"].submit(cmd("d4", "arm_4", T.MOVE, target=same))
    assert r.status == S.REJECTED and r.detail.startswith("predicted_collision_with: arm_1")
    assert world.run_until_idle(20) and world.arms["arm_1"].result("d1").status == S.COMPLETED


def test_d_runtime_crossing_is_stopped(world):
    world.arms["arm_1"].submit(cmd("x1", "arm_1", T.MOVE, target=Pose((-0.15, 0.0, 0.12))))
    world.arms["arm_4"].submit(cmd("x4", "arm_4", T.MOVE, target=Pose((0.15, 0.0, 0.12))))
    world.run_until_idle(20)
    for arm, eid in ((world.arms["arm_1"], "x1"), (world.arms["arm_4"], "x4")):
        r = arm.result(eid)
        assert r.status == S.FAILED and r.detail.split(":")[0] in ("separation_violation", "collision")
    assert world.telemetry.events_of("separation_violation") or world.telemetry.events_of("collision")


# ------------------------------------------------ (e) errores de agarre y comunicacion
def test_e_grasp_failure_is_reported_not_faked(world):
    arm = world.arms["arm_1"]
    arm.submit(cmd("g", "arm_1", T.GRASP, object_id="obj_1"))
    world.run_for(0.9)  # el brazo ya se aproxima
    world.inject_object_displacement("obj_1", (0.10, 0.12, 0.02))  # el objeto "se escapa"
    world.run_until_idle(20)
    r = arm.result("g")
    assert r.status == S.FAILED and r.detail.startswith("grasp_failed")
    assert arm.held is None and world.objects["obj_1"].held_by == ""


def test_e_comm_loss_marks_state_invalid_and_rejects(world):
    arm = world.arms["arm_1"]
    world.inject_comm_loss("arm_1", 1.0)
    s = arm.state()
    assert not s.valid and s.status == "comm_lost" and not s.available
    assert arm.submit(cmd("c1", "arm_1", T.MOVE, target=Pose((0.15, 0, 0.25)))).detail == "comm_lost"
    world.run_for(1.1)
    assert arm.state().valid
    assert arm.execute(cmd("c2", "arm_1", T.MOVE, target=Pose((0.15, 0, 0.25)))).status == S.COMPLETED


def test_no_false_success_when_target_unreachable(world):
    # Dentro de la esfera de alcance (pasa la validacion) pero sobre la base: herramienta abajo inviable.
    r = world.arms["arm_1"].execute(cmd("u", "arm_1", T.MOVE, target=Pose((0.6, 0.0, 0.30))))
    assert r.status == S.FAILED and r.detail.startswith("target_not_reached") and r.error_m > 0.01


@pytest.mark.parametrize("name,command,reason", [
    ("lejos", cmd("r1", "arm_1", T.MOVE, target=Pose((-0.6, 0.0, 0.2))), "out_of_reach"),
    ("pesado", cmd("r2", "arm_1", T.GRASP, object_id="heavy_block"), "payload_exceeded"),
    ("suelo", cmd("r3", "arm_1", T.MOVE, target=Pose((0.1, 0.0, -0.05))), "below_floor"),
    ("objeto", cmd("r4", "arm_1", T.GRASP, object_id="nope"), "unknown_object"),
    ("sin_objeto", cmd("r5", "arm_1", T.SORT, target=Pose((0.3, 0, 0.02))), "not_holding_object"),
    ("frame", cmd("r6", "arm_1", T.MOVE, target=Pose((0.1, 0, 0.2), frame="arm_1/base")), "unsupported_frame"),
])
def test_rejections_have_explicit_reasons(world, name, command, reason):
    r = world.arms["arm_1"].submit(command)
    assert r.status == S.REJECTED and r.detail.startswith(reason)


def test_busy_arm_rejects_second_command_and_cancel_works(world):
    arm = world.arms["arm_1"]
    arm.submit(cmd("b1", "arm_1", T.MOVE, target=Pose((0.15, 0, 0.25)), max_speed=0.1))
    assert arm.submit(cmd("b2", "arm_1", T.MOVE, target=Pose((0.1, 0, 0.2)))).detail.startswith("busy")
    world.run_for(0.3)
    assert arm.cancel("b1") and arm.result("b1").status == S.CANCELLED
    p0 = arm.tcp()[0].copy()
    world.run_for(0.5)
    assert np.linalg.norm(arm.tcp()[0] - p0) < 0.005  # queda parado, no sigue moviendose


# --------------------------------------------------------- (f) determinismo
def _scripted_run(w):
    a1, a4 = w.arms["arm_1"], w.arms["arm_4"]
    a1.submit(cmd("g1", "arm_1", T.GRASP, object_id="obj_1"))
    a4.submit(cmd("g4", "arm_4", T.GRASP, object_id="obj_2"))
    w.run_until_idle(30)
    a1.submit(cmd("s1", "arm_1", T.SORT, target=Pose((0.32, 0.0, 0.022))))
    w.run_until_idle(30)
    return w.telemetry.digest()


def test_f_deterministic_replay():
    w1, w2 = SimWorld(CONFIG), SimWorld(CONFIG)
    d1, d2 = _scripted_run(w1), _scripted_run(w2)
    assert d1 == d2
    w1.reset()  # tambien tras reiniciar el mismo mundo
    assert _scripted_run(w1) == d1
    w1.close(), w2.close()


def test_telemetry_has_timestamps_and_samples(world, tmp_path):
    world.arms["arm_1"].execute(cmd("t", "arm_1", T.MOVE, target=Pose((0.15, 0, 0.25))))
    assert world.telemetry.samples and all("sim_time" in s for s in world.telemetry.samples)
    times = [e["sim_time"] for e in world.telemetry.events]
    assert times == sorted(times)
    world.telemetry.save_jsonl(tmp_path / "t.jsonl")
    assert (tmp_path / "t.jsonl").stat().st_size > 0


# ---------------------------------------------------------------- pasarela
def test_gateway_roundtrip_over_tcp(world):
    server = make_server(world, port=0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        c = GatewayClient(port=server.server_address[1])
        hello = c.call("hello")
        assert hello["protocol"].startswith("swarm-sim") and len(hello["robots"]) == 6
        r = c.call("submit", command={"execution_id": "n1", "robot_id": "arm_1", "type": "Move",
                                      "target": {"x": 0.15, "y": 0.0, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}})
        assert r["status"] == "accepted"
        assert c.call("run_until_idle", timeout_s=30)["idle"]
        assert c.call("status", robot_id="arm_1", execution_id="n1")["status"] == "completed"
        assert len(c.call("states")) == 6 and len(c.call("observe")["objects"]) == 4
        with pytest.raises(RuntimeError):
            c.call("op_inexistente")
        c.close()
    finally:
        server.shutdown()
        server.server_close()

"""Validaciones de configuración sin abrir GUI."""
import json
from pathlib import Path

CFG = Path(__file__).resolve().parents[1] / "config" / "scenario.json"

def test_scale_and_reach():
    cfg = json.loads(CFG.read_text())
    m = cfg["robot_models"]["panda"]
    assert abs(m["urdf_global_scaling"] * 0.78 - 0.43) < 1e-6
    assert m["reach_m"] == 0.43
    assert len(cfg["stations"]) == 6
    assert cfg["hexagon"]["shoulder_mount_radius_m"] == 0.36
    assert 2*m["gripper_open_m"] >= cfg["objects"][0]["size"][0]

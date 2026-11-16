import math
from pathlib import Path
import pytest
from swarm_sim.world import SimWorld

CFG = Path(__file__).resolve().parents[1] / "config" / "scenario.json"

def test_progressive_rotation_and_completion():
    w = SimWorld(CFG)
    try:
        w.request_platform_rotation(math.pi / 3)
        assert w.platform_moving
        w.step(1)
        assert 0 < w.platform_angle < math.pi / 3
        w.run_for(3.2)
        assert not w.platform_moving
        assert math.isclose(w.platform_angle, math.pi / 3, abs_tol=1e-8)
    finally:
        w.close()

def test_registered_cargo_moves_with_ring():
    w = SimWorld(CFG)
    try:
        obj = next(iter(w.objects.values()))
        w.bc.resetBasePositionAndOrientation(obj.body, [0.39, 0, 0.05], [0,0,0,1])
        w.register_platform_cargo(obj.id)
        w.request_platform_rotation(math.pi / 2)
        w.run_for(4.6)
        pos = obj.position()
        assert abs(pos[0]) < 0.02 and abs(pos[1] - 0.39) < 0.02
    finally:
        w.close()

def test_invalid_cargo_and_angle():
    w = SimWorld(CFG)
    try:
        obj = next(iter(w.objects.values()))
        with pytest.raises(ValueError):
            w.register_platform_cargo(obj.id)
        with pytest.raises(ValueError):
            w.request_platform_rotation(float("nan"))
    finally:
        w.close()

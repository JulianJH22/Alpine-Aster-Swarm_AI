import math
from pathlib import Path
import pytest
from swarm_sim.world import SimWorld

CFG = Path(__file__).resolve().parents[1] / "config" / "scenario.json"


def test_two_surfaces_and_six_sectors():
    w = SimWorld(CFG)
    try:
        assert len(w.platform_sector_bodies) == 6
        assert len(w.arms) == 6
        assert len(w.zones) == 6
        assert math.isclose(w.platform_inner_radius, .205)
        assert math.isclose(w.cfg["sorting_platform"]["work_radius_m"], .2)
        w.rotate_platform(math.pi/3)
        assert math.isclose(w.platform_angle, math.pi/3)
        assert math.isclose(next(iter(w.zones.values())).center[0], .39*math.cos(math.pi/2), abs_tol=1e-6)
    finally:
        w.close()


def test_nonfinite_rotation_rejected():
    w=SimWorld(CFG)
    try:
        with pytest.raises(ValueError):
            w.rotate_platform(float('nan'))
    finally:
        w.close()

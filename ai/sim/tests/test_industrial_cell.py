from pathlib import Path
import math
from swarm_sim.world import SimWorld

CFG = Path(__file__).resolve().parents[1] / "config" / "scenario.json"

def test_six_arms_six_side_conveyors_and_two_rails():
    w = SimWorld(CFG)
    try:
        assert len(w.arms) == 6
        assert len(w.platform_sector_bodies) == 6
        assert len(w.conveyor_centers) == 6
        assert len(w.vertex_mounts) == 6
        assert math.isclose(w.camera_rail_radius, w.cfg["sorting_platform"]["work_radius_m"])
        assert w.camera_rail_radius < w.tool_rail_radius
        assert all(math.isclose(math.hypot(*c[:2]), math.hypot(*w.conveyor_centers[0][:2])) for c in w.conveyor_centers)
        w.request_platform_rotation(math.pi/6)
        w.step(5)
        assert w.platform_angle > 0
    finally:
        w.close()

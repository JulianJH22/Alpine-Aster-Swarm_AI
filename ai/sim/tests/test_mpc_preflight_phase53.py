"""Pruebas del preplanificador, sin requerir PyBullet."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('mpc_preflight', Path(__file__).resolve().parents[1] / 'swarm_sim/mpc_preflight.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def plan(target, others=(), start=(0, 0, .2)):
    return mod.plan_tcp(start, target, (0, 0, 0), .43, others, .15, 0, .1, 1/240)


def test_safe_short_motion():
    r = plan((.1, 0, .2))
    assert r['feasible_preflight'] and len(r['waypoints']) == 12


def test_reject_out_of_reach():
    assert plan((.6, 0, .2))['reason'] == 'out_of_reach'


def test_reject_swept_tcp_collision():
    assert plan((.2, 0, .2), [(0.1, 0, .2)])['reason'] == 'tcp_separation_risk'


def test_reject_below_floor():
    assert plan((.1, 0, -.1))['reason'] == 'below_floor'


def test_reject_invalid_input():
    import pytest
    with pytest.raises(ValueError):
        plan((float('nan'), 0, .2))

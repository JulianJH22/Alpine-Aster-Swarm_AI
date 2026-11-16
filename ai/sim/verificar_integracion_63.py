"""Verifica que demo y aprendizaje cargan exactamente el mismo escenario."""
import json
from pathlib import Path
from learning.environment import SwarmLearningEnv
from swarm_sim.world import SimWorld

SCENARIO = Path(__file__).resolve().parent / 'config' / 'scenario.json'

def main():
    cfg = json.loads(SCENARIO.read_text(encoding='utf-8'))
    assert cfg['industrial_cell']['enabled'] is True
    assert cfg['industrial_cell']['conveyor_count'] == 6
    with_world = SimWorld(SCENARIO, gui=False, realtime=False)
    try:
        assert len(with_world.arms) == 6
        assert len(with_world.conveyor_centers) == 6
        expected = [(k, tuple(a.base_pos)) for k, a in with_world.arms.items()]
    finally:
        with_world.close()
    env = SwarmLearningEnv(num_agents=1, scenario=SCENARIO, gui=False)
    try:
        actual = [(k, tuple(a.base_pos)) for k, a in env.world.arms.items()]
        assert actual == expected, (actual, expected)
        assert len(env.world.conveyor_centers) == 6
        print('INTEGRACION OK: escenario, seis brazos, seis cintas y posiciones idénticas')
        print('NOTA: no demuestra que el objetivo de entrenamiento sea alcanzable.')
    finally:
        env.close()

if __name__ == '__main__':
    main()

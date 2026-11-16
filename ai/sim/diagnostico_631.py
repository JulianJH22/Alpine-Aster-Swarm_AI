"""Diagnostico del entorno: compara control hacia meta con acciones nulas.

No entrena redes. Comprueba si las ordenes XYZ producen progreso real.
"""
import argparse
import json
import numpy as np
from learning.environment import SwarmLearningEnv
from learning.config import LearningConfig


def trial(seed, controller, steps=120):
    env = SwarmLearningEnv(num_agents=1, seed=seed, config=LearningConfig(goal_radius_m=0.025))
    try:
        obs, _ = env.reset(seed=seed)
        aid = env.agents[0]
        start = float(np.linalg.norm(env.goals[aid]-env.world.arms[aid].tcp()[0]))
        invalid = 0
        for _ in range(steps):
            if controller == 'target':
                error = env.goals[aid]-env.world.arms[aid].tcp()[0]
                action = np.clip(error / env.cfg.action_scale_m, -1, 1)
            else:
                action = np.zeros(3)
            _, _, done, trunc, info = env.step({aid: action})
            invalid += int(info['invalid_actions'][aid])
            if done[aid] or trunc[aid]:
                break
        end = float(info['distances_m'][aid])
        return dict(seed=seed, controller=controller, initial_m=start, final_m=end,
                    reduction_m=start-end, success=bool(done[aid]), invalid=invalid)
    finally:
        env.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seeds', type=int, default=5)
    parser.add_argument('--out', default='out/phase631_diagnostic.json')
    args = parser.parse_args()
    from pathlib import Path
    results = [trial(4200+i, mode) for i in range(args.seeds) for mode in ('zero','target')]
    output = Path(args.out); output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(results, indent=2))
    for mode in ('zero','target'):
        group = [r for r in results if r['controller'] == mode]
        print(mode, 'success', sum(r['success'] for r in group), '/', len(group),
              'mean_reduction_m', round(float(np.mean([r['reduction_m'] for r in group])), 5),
              'invalid', sum(r['invalid'] for r in group))
    print('Datos:', output)


if __name__ == '__main__':
    main()

"""Prueba de humo sin entrenamiento. Ejecutar con PyBullet instalado."""
import argparse
import numpy as np
from learning import SwarmLearningEnv

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--agents', type=int, choices=[1,2,6], default=1)
    parser.add_argument('--steps', type=int, default=20)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    env = SwarmLearningEnv(num_agents=args.agents, seed=args.seed)
    try:
        obs, info = env.reset(seed=args.seed)
        assert all(o.shape == (14,) for o in obs.values())
        print('Agentes:', list(obs), 'Objetivos:', info['goals'])
        for i in range(args.steps):
            actions = {aid: np.zeros(3) for aid in env.agents}
            obs, rewards, terminated, truncated, info = env.step(actions)
            print(f"step={i+1} t={info['sim_time']:.2f} recompensas={rewards}")
            if all(terminated[a] or truncated[a] for a in env.agents):
                break
        print('OK: simulacion avanzada sin intervención manual')
    finally:
        env.close()

if __name__ == '__main__':
    main()

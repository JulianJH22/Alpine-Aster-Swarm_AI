"""FASE 6.2: smoke test y entrenamiento MAPPO configurable.

No lanza automaticamente el curriculum de 1000 episodios de FASE 6.3.
"""
import argparse
import csv
from pathlib import Path
import random
import numpy as np
import torch
from learning.environment import SwarmLearningEnv
from learning.mappo import MAPPO, PPOConfig, encode, gae, ppo_update


def run(agents=1, episodes=2, seed=42, out='out/phase62', update=True):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    dest = Path(out)
    dest.mkdir(parents=True, exist_ok=True)
    model = MAPPO()
    cfg = PPOConfig()
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg.learning_rate)
    env = SwarmLearningEnv(num_agents=agents, platform_rotation=False, seed=seed, gui=False)
    history = []
    try:
        for episode in range(episodes):
            obs, info = env.reset(seed=seed + episode)
            ids = list(env.agents)
            records = []
            episode_reward = 0.0
            while True:
                state = encode(obs, ids)
                actor_obs = np.stack([obs[a] for a in ids])
                with torch.no_grad():
                    action, logp = model.act(torch.from_numpy(actor_obs))
                    value = float(model.value(torch.from_numpy(state)).item())
                actions = {a: action[i].numpy() for i, a in enumerate(ids)}
                next_obs, rewards, terminated, truncated, info = env.step(actions)
                done = all(terminated[a] or truncated[a] for a in ids)
                # Recompensa global compartida: optimiza la cooperacion.
                team_reward = float(sum(rewards.values()))
                episode_reward += team_reward
                with torch.no_grad():
                    next_value = float(model.value(torch.from_numpy(encode(next_obs, ids))).item())
                records.append((state, actor_obs, action.numpy(), logp.numpy(), value, next_value, team_reward, float(done)))
                obs = next_obs
                if done:
                    break
            # GAE sobre la trayectoria conjunta: un valor global por paso.
            rew = np.array([[r[6]] for r in records], dtype=np.float32)
            val = np.array([[r[4]] for r in records], dtype=np.float32)
            nxt = np.array([[r[5]] for r in records], dtype=np.float32)
            dones = np.array([[r[7]] for r in records], dtype=np.float32)
            advantage, returns = gae(rew, val, nxt, dones, cfg.gamma, cfg.gae_lambda)
            batch = {
                'global_states': np.stack([r[0] for r in records]),
                'returns': returns[:, 0],
                'actor_obs': np.concatenate([r[1] for r in records]),
                'actions': np.concatenate([r[2] for r in records]),
                'old_logp': np.concatenate([r[3] for r in records]),
                'advantages': np.repeat(advantage[:, 0], agents),
            }
            metrics = ppo_update(model, optimizer, batch, cfg) if update else {}
            result = {'episode': episode + 1, 'agents': agents, 'steps': len(records),
                      'team_reward': episode_reward, 'successes': sum(info['distances_m'][a] <= env.cfg.success_tolerance_m for a in ids), **metrics}
            history.append(result)
            print(result, flush=True)
            torch.save({'model': model.state_dict(), 'optimizer': optimizer.state_dict(),
                        'episode': episode + 1, 'seed': seed, 'agents': agents,
                        'ppo_config': vars(cfg)}, dest / 'checkpoint_latest.pt')
        with (dest / 'metrics.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=history[0].keys())
            writer.writeheader()
            writer.writerows(history)
    finally:
        env.close()
    return history


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--agents', type=int, choices=(1, 2, 6), default=1)
    parser.add_argument('--episodes', type=int, default=2)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--out', default='out/phase62')
    parser.add_argument('--no-update', action='store_true')
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error('--episodes debe ser positivo')
    run(args.agents, args.episodes, args.seed, args.out, not args.no_update)

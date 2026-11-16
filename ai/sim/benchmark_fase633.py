"""Benchmark no invasivo de Fase 6.3.3. No modifica los algoritmos existentes."""
import argparse
import json
import statistics
import time
from pathlib import Path
from unittest.mock import patch

import numpy as np
import torch
from learning.environment import SwarmLearningEnv
from learning.mappo import MAPPO, PPOConfig
from train_mappo_63 import episode, update


def run(agents, episodes, seed, output, torch_threads):
    torch.set_num_threads(torch_threads)
    torch.manual_seed(seed)
    model = MAPPO()
    config = PPOConfig()
    optimizer = torch.optim.Adam(model.parameters(), lr=config.learning_rate)
    timings = {'reset': [], 'physics_and_environment_step': [], 'actor_inference': [],
               'critic_inference': [], 'ppo_update': [], 'episode_total': []}
    def timed(name, fn):
        def wrapper(*args, **kwargs):
            start = time.perf_counter()
            try:
                return fn(*args, **kwargs)
            finally:
                timings[name].append(time.perf_counter() - start)
        return wrapper
    start_all = time.perf_counter()
    env = SwarmLearningEnv(num_agents=agents, seed=seed, gui=False)
    metrics = []
    try:
        with patch.object(env, 'reset', timed('reset', env.reset)), \
             patch.object(env, 'step', timed('physics_and_environment_step', env.step)), \
             patch.object(model, 'act', timed('actor_inference', model.act)), \
             patch.object(model, 'value', timed('critic_inference', model.value)):
            for i in range(episodes):
                start = time.perf_counter()
                records, info = episode(env, model, seed + i, train=True)
                timings['episode_total'].append(time.perf_counter() - start)
                start = time.perf_counter()
                update(model, optimizer, records, agents, config)
                timings['ppo_update'].append(time.perf_counter() - start)
                metrics.append(info)
    finally:
        env.close()
    elapsed = time.perf_counter() - start_all
    summary = {name: {'calls': len(v), 'total_s': sum(v),
                      'mean_ms': 1000 * statistics.mean(v) if v else 0}
               for name, v in timings.items()}
    result = {'agents': agents, 'episodes': episodes, 'seed': seed,
              'torch_threads': torch_threads, 'device': 'cpu',
              'elapsed_total_s': elapsed, 'timings': summary,
              'steps_total': sum(m['steps'] for m in metrics),
              'team_success_rate': float(np.mean([m['team_success'] for m in metrics])),
              'note': 'Los tiempos por categoria pueden solaparse: episode_total incluye reset, step e inferencias.'}
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--agents', type=int, choices=[1, 2, 6], default=1)
    parser.add_argument('--episodes', type=int, default=3)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--torch-threads', type=int, default=1)
    parser.add_argument('--output', default='out/benchmark_fase633.json')
    args = parser.parse_args()
    if args.episodes < 1 or args.torch_threads < 1:
        parser.error('episodes y torch-threads deben ser positivos')
    run(**vars(args))

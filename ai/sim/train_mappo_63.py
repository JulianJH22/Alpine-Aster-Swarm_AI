"""FASE 6.3: calibracion, curriculum MAPPO y evaluacion sin exploracion.

El actor es compartido (una politica replicable en seis instancias); el critico
centralizado solo se utiliza para entrenar. Plataforma fija.
"""
import argparse
import csv
import json
import random
from dataclasses import replace
from pathlib import Path

import numpy as np
import torch
from learning.environment import SwarmLearningEnv
from learning.config import LearningConfig
from learning.mappo import MAPPO, PPOConfig, encode, gae, ppo_update

STAGES = [(1, 300), (2, 300), (6, 400)]
FIELDS = ['phase','stage','episode','agents','steps','team_reward','reward_per_agent',
          'successes','team_success','invalid_actions','final_distance_mean_m',
          'progress_reward','time_penalty','invalid_penalty','success_bonus',
          'actor_loss','value_loss','initial_distance_mean_m','distance_reduction_mean_m',
          'distance_reduction_pct','eval_count','goal_radius_m']


def episode(env, model, seed, train=True):
    obs, _ = env.reset(seed=seed)
    ids = list(env.agents)
    records = []
    sums = dict(progress_reward=0., time_penalty=0., invalid_penalty=0., success_bonus=0.)
    total = 0.
    invalid_count = 0
    initial_distances = {a: float(np.linalg.norm(env.goals[a]-env.world.arms[a].tcp()[0])) for a in ids}
    while True:
        state = encode(obs, ids)
        local = np.stack([obs[a] for a in ids]).astype(np.float32)
        with torch.no_grad():
            act, logp = model.act(torch.from_numpy(local), deterministic=not train)
            value = float(model.value(torch.from_numpy(state)).item())
        nxt, rewards, terminated, truncated, info = env.step({a: act[i].numpy() for i,a in enumerate(ids)})
        done = all(terminated[a] or truncated[a] for a in ids)
        reward_sum = float(sum(rewards.values()))
        total += reward_sum
        invalid_count += sum(bool(v) for v in info['invalid_actions'].values())
        for components in info['reward_components'].values():
            for k, v in [('progress_reward', 'progress'), ('time_penalty', 'time'),
                         ('invalid_penalty', 'invalid'), ('success_bonus', 'success')]:
                sums[k] += float(components[v])
        with torch.no_grad():
            next_value = float(model.value(torch.from_numpy(encode(nxt, ids))).item())
        records.append((state, local, act.numpy(), logp.numpy(), value, next_value,
                        reward_sum, float(done and not any(truncated.values()))))
        obs = nxt
        if done:
            break
    successes = sum(info['distances_m'][a] <= env.cfg.success_tolerance_m for a in ids)
    initial_mean = float(np.mean(list(initial_distances.values())))
    final_mean = float(np.mean(list(info['distances_m'].values())))
    metrics = dict(initial_distance_mean_m=initial_mean,
                   distance_reduction_mean_m=initial_mean-final_mean,
                   distance_reduction_pct=100*(initial_mean-final_mean)/max(initial_mean, 1e-9),
                   steps=len(records), team_reward=total, reward_per_agent=total/len(ids),
                   successes=successes, team_success=int(successes == len(ids)),
                   invalid_actions=invalid_count,
                   final_distance_mean_m=float(np.mean(list(info['distances_m'].values()))), **sums)
    return records, metrics


def update(model, optimizer, records, agents, cfg):
    rew = np.asarray([[r[6]] for r in records], dtype=np.float32)
    val = np.asarray([[r[4]] for r in records], dtype=np.float32)
    nxt = np.asarray([[r[5]] for r in records], dtype=np.float32)
    terminal = np.asarray([[r[7]] for r in records], dtype=np.float32)
    adv, returns = gae(rew, val, nxt, terminal, cfg.gamma, cfg.gae_lambda)
    batch = dict(global_states=np.stack([r[0] for r in records]),
                 returns=returns[:,0], actor_obs=np.concatenate([r[1] for r in records]),
                 actions=np.concatenate([r[2] for r in records]),
                 old_logp=np.concatenate([r[3] for r in records]),
                 advantages=np.repeat(adv[:,0], agents))
    return ppo_update(model, optimizer, batch, cfg)


def append_csv(path, row):
    exists = path.exists()
    with path.open('a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        if not exists:
            writer.writeheader()
        writer.writerow({k: row.get(k, '') for k in FIELDS})


def save(path, model, optimizer, stage, episode, seed, cfg):
    torch.save(dict(model=model.state_dict(), optimizer=optimizer.state_dict(),
                    stage=stage, episode=episode, seed=seed, ppo_config=vars(cfg)), path)


def run(out='out/phase63', seed=42, calibrate=False, episodes=None,
        eval_every=50, eval_episodes=5, resume=False, gui=False, torch_threads=1, bc_actor=None):
    if torch_threads < 1: raise ValueError('torch_threads debe ser >= 1')
    torch.set_num_threads(torch_threads)
    random.seed(seed); np.random.seed(seed); torch.manual_seed(seed)
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    cfg = PPOConfig()
    model = MAPPO()
    if bc_actor is not None and not resume:
        payload = torch.load(bc_actor, map_location='cpu', weights_only=False)
        if payload.get('format') != 'bc_actor_6333' or payload.get('obs_dim') != 14 or payload.get('act_dim') != 3:
            raise ValueError('Checkpoint BC incompatible')
        model.actor.load_state_dict(payload['actor'], strict=True)
        print('Actor inicializado desde Behavior Cloning:', bc_actor, flush=True)
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg.learning_rate)
    stages = [(1, 30)] if calibrate else STAGES
    if episodes is not None:
        # Modo smoke-test: mismo orden progresivo, sin ejecutar 1000 episodios.
        remaining = episodes
        stages = []
        for agents, n in ([(1,30)] if calibrate else STAGES):
            count = min(remaining, n)
            if count:
                stages.append((agents, count))
            remaining -= count
            if remaining <= 0: break
    start_stage = 0; start_episode = 0
    checkpoint = out/'checkpoint_latest.pt'
    if resume and checkpoint.exists():
        state = torch.load(checkpoint, map_location='cpu', weights_only=False)
        if state['seed'] != seed:
            raise ValueError('La semilla del checkpoint no coincide')
        model.load_state_dict(state['model']); optimizer.load_state_dict(state['optimizer'])
        start_stage = state['stage']; start_episode = state['episode']
    elif resume:
        raise FileNotFoundError(checkpoint)
    if not resume and (out/'training.csv').exists():
        raise FileExistsError(f'La carpeta {out} contiene un entrenamiento previo. Usa otra --out o --resume.')
    (out/'configuration.json').write_text(json.dumps(dict(seed=seed, stages=stages,
       calibrate=calibrate, eval_every=eval_every, eval_episodes=eval_episodes,
       shared_actor=True, platform_rotation=False, torch_threads=torch_threads, bc_actor=str(bc_actor) if bc_actor else None), indent=2))
    for stage_idx, (agents, n) in enumerate(stages):
        if stage_idx < start_stage: continue
        learning_cfg = LearningConfig(goal_radius_m=0.025 if calibrate else 0.04)
        env = SwarmLearningEnv(num_agents=agents, platform_rotation=False, seed=seed, gui=gui, config=learning_cfg)
        evaluation = SwarmLearningEnv(num_agents=agents, platform_rotation=False, seed=seed+100000, gui=False, config=learning_cfg)
        try:
            begin = start_episode if stage_idx == start_stage else 0
            for e in range(begin, n):
                records, metrics = episode(env, model, seed + stage_idx*10000 + e, True)
                metrics.update(update(model, optimizer, records, agents, cfg))
                row = dict(phase='calibration' if calibrate else 'training', stage=stage_idx+1,
                           episode=e+1, agents=agents, goal_radius_m=learning_cfg.goal_radius_m, **metrics)
                append_csv(out/'training.csv', row)
                print(row, flush=True)
                next_stage, next_episode = (stage_idx, e+1) if e+1 < n else (stage_idx+1, 0)
                save(checkpoint, model, optimizer, next_stage, next_episode, seed, cfg)
                if (e+1) % eval_every == 0 or e+1 == n:
                    results = [episode(evaluation, model, seed+100000+stage_idx*10000+j, False)[1]
                               for j in range(eval_episodes)]
                    summary = {k: float(np.mean([r[k] for r in results])) for k in results[0]}
                    append_csv(out/'evaluation.csv', dict(phase='evaluation', stage=stage_idx+1,
                               episode=e+1, agents=agents, eval_count=len(results),
                               goal_radius_m=learning_cfg.goal_radius_m, **summary))
                    for j, r in enumerate(results):
                        append_csv(out/'evaluation_detail.csv', dict(phase='evaluation_detail',
                                   stage=stage_idx+1, episode=e+1, agents=agents,
                                   eval_count=j+1, goal_radius_m=learning_cfg.goal_radius_m, **r))
                    print('EVALUATION', stage_idx+1, e+1, summary, flush=True)
                    save(out/f'checkpoint_stage{stage_idx+1}_ep{e+1}.pt', model, optimizer,
                         next_stage, next_episode, seed, cfg)
        finally:
            env.close(); evaluation.close()
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default='out/phase63')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--calibrate', action='store_true')
    parser.add_argument('--episodes', type=int, default=None, help='Limita episodios para pruebas')
    parser.add_argument('--eval-every', type=int, default=50)
    parser.add_argument('--eval-episodes', type=int, default=5)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--bc-actor', default=None, help='Checkpoint bc_actor_best.pt para inicializar el actor')
    parser.add_argument('--gui', action='store_true', help='Muestra el mismo entorno industrial durante el entrenamiento (más lento)')
    parser.add_argument('--torch-threads', type=int, default=1, help='Hilos CPU de PyTorch')
    args = parser.parse_args()
    if args.episodes is not None and args.episodes < 1: parser.error('episodes debe ser >= 1')
    if args.eval_every < 1 or args.eval_episodes < 1: parser.error('eval debe ser >= 1')
    run(**vars(args))

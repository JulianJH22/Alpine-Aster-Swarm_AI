"""Fase 6.3.3.3: demostraciones geometricas y preentrenamiento del actor MAPPO.

El profesor no es un controlador de seguridad; cada accion se valida en el entorno.
"""
import argparse
import json
import random
import time
from pathlib import Path

import numpy as np
import torch
from learning.config import LearningConfig
from learning.environment import SwarmLearningEnv
from learning.mappo import MAPPO


def teacher(obs, active, scale):
    if not active:
        return np.zeros(3, dtype=np.float32)
    # observacion: tcp_rel[0:3], goal_rel[3:6], error[6:9], velocity[9:12], idle[12], distance[13]
    delta = np.asarray(obs[6:9], dtype=np.float32)
    return np.clip(delta / scale, -0.95, 0.95).astype(np.float32)


def collect(args):
    cfg = LearningConfig(goal_radius_m=args.goal_radius)
    env = SwarmLearningEnv(num_agents=args.agents, config=cfg, seed=args.seed)
    xs, ys, stats = [], [], []
    try:
        for ep in range(args.episodes):
            obs, _ = env.reset(seed=args.seed + ep)
            initial = dict(env.initial_distances)
            invalids = 0
            while True:
                actions = {}
                for aid in env.agents:
                    arm = env.world.arms[aid]
                    idle = arm.active is None and aid not in env.finished
                    actions[aid] = teacher(obs[aid], idle, cfg.action_scale_m)
                    # Acciones durante una maniobra activa son ignoradas por environment.step.
                    # No entrenar sobre esas observaciones para evitar supervisar acciones irrelevantes.
                    if idle:
                        xs.append(np.asarray(obs[aid], dtype=np.float32).copy())
                        ys.append(actions[aid].copy())
                obs, _, terminated, truncated, info = env.step(actions)
                invalids += sum(bool(x) for x in info['invalid_actions'].values())
                if all(terminated[a] or truncated[a] for a in env.agents):
                    break
            success = sum(info['distances_m'][a] <= cfg.success_tolerance_m for a in env.agents)
            stats.append(dict(episode=ep + 1, successes=success, team_success=int(success == len(env.agents)),
                              invalid=invalids, final_distance_mean_m=float(np.mean(list(info['distances_m'].values()))),
                              initial_distance_mean_m=float(np.mean(list(initial.values())))))
            if (ep + 1) % 10 == 0 or ep + 1 == args.episodes:
                print(f'collect {ep+1}/{args.episodes} samples={len(xs)} team_success={np.mean([s["team_success"] for s in stats]):.3f}', flush=True)
    finally:
        env.close()
    if not xs:
        raise RuntimeError('No se obtuvieron demostraciones: revisar disponibilidad de los brazos')
    return np.stack(xs), np.stack(ys), stats


def fit(args, x, y):
    torch.set_num_threads(args.torch_threads)
    torch.manual_seed(args.seed)
    model = MAPPO()
    # Solo entrenamos el actor: el critico se inicializa posteriormente con PPO.
    optimizer = torch.optim.Adam(model.actor.parameters(), lr=args.lr)
    rng = np.random.default_rng(args.seed)
    order = rng.permutation(len(x))
    nval = max(1, int(len(x) * 0.2)) if len(x) > 1 else 0
    train_ids, val_ids = order[nval:], order[:nval]
    if not len(train_ids):
        raise RuntimeError('Insuficientes demostraciones')
    device = torch.device('cuda' if args.cuda and torch.cuda.is_available() else 'cpu')
    model.to(device)
    xt = torch.as_tensor(x, dtype=torch.float32, device=device)
    yt = torch.as_tensor(y, dtype=torch.float32, device=device)
    best = float('inf')
    history = []
    for epoch in range(args.epochs):
        shuffled = rng.permutation(train_ids)
        model.train()
        losses = []
        for start in range(0, len(shuffled), args.batch):
            ids = shuffled[start:start + args.batch]
            prediction = torch.tanh(model.actor(xt[ids]))
            loss = (prediction - yt[ids]).square().mean()
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()
            losses.append(loss.item())
        model.eval()
        with torch.no_grad():
            val = (torch.tanh(model.actor(xt[val_ids])) - yt[val_ids]).square().mean().item() if nval else float(np.mean(losses))
        history.append(dict(epoch=epoch + 1, train_mse=float(np.mean(losses)), val_mse=val))
        if val < best:
            best = val
            torch.save(dict(actor={k: v.detach().cpu() for k,v in model.actor.state_dict().items()},
                            obs_dim=14, act_dim=3, seed=args.seed, samples=len(x),
                            validation_mse=val, format='bc_actor_6333'), args.out / 'bc_actor_best.pt')
        print(f'epoch {epoch+1}/{args.epochs} train_mse={np.mean(losses):.6f} val_mse={val:.6f}', flush=True)
    return history, str(device)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', type=Path, default=Path('out/bc6333'))
    p.add_argument('--agents', type=int, choices=[1,2,6], default=1)
    p.add_argument('--episodes', type=int, default=40)
    p.add_argument('--epochs', type=int, default=20)
    p.add_argument('--batch', type=int, default=256)
    p.add_argument('--lr', type=float, default=3e-4)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--goal-radius', type=float, default=0.04)
    p.add_argument('--torch-threads', type=int, default=1)
    p.add_argument('--cuda', action='store_true')
    args = p.parse_args()
    if min(args.episodes, args.epochs, args.batch, args.torch_threads) < 1: p.error('Los recuentos deben ser positivos')
    args.out.mkdir(parents=True, exist_ok=True)
    random.seed(args.seed); np.random.seed(args.seed)
    start = time.perf_counter()
    x,y,stats = collect(args)
    np.savez_compressed(args.out / 'demonstrations.npz', obs=x, actions=y)
    history,device = fit(args,x,y)
    summary = dict(samples=len(x), agents=args.agents, episodes=args.episodes, device=device,
                   collection_team_success_rate=float(np.mean([s['team_success'] for s in stats])),
                   collection_mean_invalid=float(np.mean([s['invalid'] for s in stats])),
                   elapsed_s=time.perf_counter()-start, history=history, episodes_detail=stats)
    (args.out / 'report.json').write_text(json.dumps(summary, indent=2))
    print('SAVED', args.out / 'bc_actor_best.pt', flush=True)

if __name__ == '__main__':
    main()

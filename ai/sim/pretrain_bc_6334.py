"""Fase 6.3.3.4: BC sintetico consistente con el espacio de acciones MAPPO.

No reemplaza la validacion fisica: los ejemplos sinteticos son supervisiones geometricas.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import torch
from learning.mappo import MAPPO
from learning.config import LearningConfig
from pretrain_bc_6333 import collect, teacher


def synthesize(anchors, count, radius, scale, seed):
    rng = np.random.default_rng(seed)
    ids = rng.integers(0, len(anchors), count)
    x = anchors[ids].copy()
    # Alterar simultaneamente goal_rel, error y distancia para mantener coherencia.
    directions = rng.normal(size=(count, 3)).astype(np.float32)
    directions /= np.maximum(np.linalg.norm(directions, axis=1, keepdims=True), 1e-8)
    magnitudes = rng.uniform(0.002, radius, (count, 1)).astype(np.float32)
    errors = directions * magnitudes
    x[:, 6:9] = errors
    x[:, 3:6] = x[:, 0:3] + errors
    x[:, 9:12] = 0.0  # Estados estacionarios, como en decisiones del profesor
    x[:, 12] = 1.0
    x[:, 13] = np.linalg.norm(errors, axis=1)
    y = np.clip(errors / scale, -0.95, 0.95).astype(np.float32)
    return x, y


def train(args, xtr, ytr, xval, yval):
    torch.manual_seed(args.seed)
    torch.set_num_threads(args.torch_threads)
    model = MAPPO()
    opt = torch.optim.Adam(model.actor.parameters(), lr=args.lr)
    xt, yt = torch.from_numpy(xtr), torch.from_numpy(ytr)
    xv, yv = torch.from_numpy(xval), torch.from_numpy(yval)
    rng = np.random.default_rng(args.seed)
    best = float('inf'); wait = 0; history = []
    for ep in range(args.epochs):
        model.train(); losses=[]
        for ix in np.array_split(rng.permutation(len(xt)), max(1, (len(xt)+args.batch-1)//args.batch)):
            prediction = torch.tanh(model.actor(xt[ix]))
            loss = (prediction-yt[ix]).square().mean()
            opt.zero_grad(set_to_none=True); loss.backward(); opt.step()
            losses.append(float(loss.detach()))
        model.eval()
        with torch.no_grad():
            val = float((torch.tanh(model.actor(xv))-yv).square().mean())
        history.append({'epoch':ep+1,'train_mse':float(np.mean(losses)),'val_mse':val})
        if val < best - 1e-7:
            best=val; wait=0
            torch.save({'actor':{k:v.detach().cpu() for k,v in model.actor.state_dict().items()},
                        'obs_dim':14,'act_dim':3,'seed':args.seed,'samples':len(xtr),
                        'validation_mse':val,'format':'bc_actor_6333',
                        'source':'synthetic_bc_6334'},args.out/'bc_actor_best.pt')
        else:
            wait+=1
        if (ep+1)%5==0 or ep==0:
            print(f'epoch {ep+1} train={np.mean(losses):.6f} val={val:.6f}',flush=True)
        if wait>=args.patience: break
    return history,best


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--out',type=Path,default=Path('out/bc6334'))
    p.add_argument('--agents',type=int,choices=[1,2,6],default=1)
    p.add_argument('--episodes',type=int,default=40)
    p.add_argument('--samples',type=int,default=10000)
    p.add_argument('--epochs',type=int,default=60)
    p.add_argument('--batch',type=int,default=256)
    p.add_argument('--lr',type=float,default=1e-3)
    p.add_argument('--patience',type=int,default=12)
    p.add_argument('--goal-radius',type=float,default=0.04)
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--torch-threads',type=int,default=1)
    p.add_argument('--reuse-anchors',type=Path,default=None,help='NPZ con obs generado anteriormente')
    args=p.parse_args()
    if min(args.samples,args.epochs,args.batch,args.patience,args.torch_threads)<1:p.error('Valores positivos requeridos')
    args.out.mkdir(parents=True,exist_ok=True)
    t=time.perf_counter()
    if args.reuse_anchors:
        anchors=np.load(args.reuse_anchors)['obs'].astype(np.float32)
        stats=[]
    else:
        anchors,_,stats=collect(args)
    if len(anchors)<5: raise RuntimeError('Se necesitan al menos cinco observaciones de referencia')
    rng=np.random.default_rng(args.seed)
    perm=rng.permutation(len(anchors)); nval=max(1,int(len(anchors)*0.2))
    # Separacion por ancla ANTES de generar variaciones: evita fuga directa train/val.
    cfg=LearningConfig()
    tr,yr=synthesize(anchors[perm[nval:]],args.samples,args.goal_radius,cfg.action_scale_m,args.seed+10)
    va,yv=synthesize(anchors[perm[:nval]],max(500,args.samples//5),args.goal_radius,cfg.action_scale_m,args.seed+11)
    np.savez_compressed(args.out/'demonstrations.npz',obs=tr,actions=yr)
    history,best=train(args,tr,yr,va,yv)
    report={'samples':len(tr),'validation_samples':len(va),'anchor_samples':len(anchors),
            'agents':args.agents,'collection_team_success_rate':float(np.mean([s['team_success'] for s in stats])) if stats else None,
            'collection_mean_invalid':float(np.mean([s['invalid'] for s in stats])) if stats else None,
            'best_validation_mse':best,'elapsed_s':time.perf_counter()-t,
            'synthetic_data_note':'Estados geometricos sinteticos, no trayectorias validadas por PyBullet',
            'history':history}
    (args.out/'report.json').write_text(json.dumps(report,indent=2))
    print('SAVED',args.out/'bc_actor_best.pt',flush=True)

if __name__=='__main__':main()

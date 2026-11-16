"""Evaluacion fisica independiente: teacher, actor BC y mezcla de ambos."""
import argparse
import json
from pathlib import Path
import numpy as np
import torch
from learning.config import LearningConfig
from learning.environment import SwarmLearningEnv
from learning.mappo import MAPPO
from pretrain_bc_6333 import teacher


def run(env, model, episodes, seed, alpha):
    rows=[]
    for ep in range(episodes):
        obs,_=env.reset(seed=seed+ep)
        invalid=0
        while True:
            actions={}
            for aid in env.agents:
                arm=env.world.arms[aid]
                idle=arm.active is None and aid not in env.finished
                expert=teacher(obs[aid],idle,env.cfg.action_scale_m)
                if idle and alpha>0:
                    with torch.no_grad():
                        prediction=torch.tanh(model.actor(torch.from_numpy(obs[aid]))).numpy()
                    actions[aid]=np.clip((1-alpha)*expert+alpha*prediction,-1,1)
                else:actions[aid]=expert
            obs,_,terminated,truncated,info=env.step(actions)
            invalid+=sum(bool(v) for v in info['invalid_actions'].values())
            if all(terminated[a] or truncated[a] for a in env.agents):break
        distances=list(info['distances_m'].values())
        rows.append({'success':int(all(d<=env.cfg.success_tolerance_m for d in distances)),
                     'final_distance_mean_m':float(np.mean(distances)),
                     'invalid':invalid,'steps':env.steps})
    return {'team_success_rate':float(np.mean([r['success'] for r in rows])),
            'final_distance_mean_m':float(np.mean([r['final_distance_mean_m'] for r in rows])),
            'invalid_mean':float(np.mean([r['invalid'] for r in rows])),
            'steps_mean':float(np.mean([r['steps'] for r in rows])),'episodes':rows}


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--out',type=Path,default=Path('out/bc6334/evaluation.json'))
    p.add_argument('--agents',type=int,choices=[1,2,6],default=1)
    p.add_argument('--episodes',type=int,default=10)
    p.add_argument('--seed',type=int,default=1000)
    args=p.parse_args()
    torch.set_num_threads(1)
    ck=torch.load(args.checkpoint,map_location='cpu',weights_only=False)
    model=MAPPO();model.actor.load_state_dict(ck['actor']);model.eval()
    env=SwarmLearningEnv(num_agents=args.agents,config=LearningConfig(),seed=args.seed)
    try:
        results={name:run(env,model,args.episodes,args.seed,alpha) for name,alpha in
                 [('teacher',0.0),('hybrid_25pct_actor',0.25),('hybrid_50pct_actor',0.5),('bc_actor',1.0)]}
    finally:env.close()
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(results,indent=2))
    for name,r in results.items():print(name,'success=',r['team_success_rate'],'distance=',r['final_distance_mean_m'],'invalid=',r['invalid_mean'],flush=True)
    print('SAVED',args.out)

if __name__=='__main__':main()

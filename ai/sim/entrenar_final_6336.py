#!/usr/bin/env python3
"""Entrenamiento final MAPPO: transferencia de pesos, etapas y seleccion robusta.
No modifica el proceso nocturno 6335 ni sus checkpoints.
"""
import argparse, csv, json, os, random, time
from pathlib import Path
import numpy as np
import torch
from learning.environment import SwarmLearningEnv
from learning.config import LearningConfig
from learning.mappo import MAPPO, PPOConfig
from train_mappo_63 import episode, update, append_csv, save


def evaluate(env, model, seeds):
    results = [episode(env, model, s, train=False)[1] for s in seeds]
    return {k: float(np.mean([r[k] for r in results])) for k in results[0]}


def main():
    p=argparse.ArgumentParser(description='Entrenamiento final multibrazo con checkpoints y presupuesto temporal')
    p.add_argument('--source', required=True, help='checkpoint_latest.pt del entrenamiento anterior')
    p.add_argument('--out', default='out/final6336')
    p.add_argument('--hours', type=float, default=8)
    p.add_argument('--seed', type=int, default=20261116)
    p.add_argument('--stages', default='1:100,2:350,6:650', help='agentes:episodios, separados por comas')
    p.add_argument('--eval-every', type=int, default=25)
    p.add_argument('--eval-episodes', type=int, default=12)
    p.add_argument('--final-eval-episodes', type=int, default=30)
    p.add_argument('--learning-rate', type=float, default=0.0001)
    p.add_argument('--torch-threads', type=int, default=1)
    p.add_argument('--resume', action='store_true')
    a=p.parse_args()
    if min(a.hours,a.eval_every,a.eval_episodes,a.final_eval_episodes,a.learning_rate,a.torch_threads)<=0:p.error('Parametros deben ser positivos')
    try: stages=[tuple(map(int,s.split(':'))) for s in a.stages.split(',')]
    except ValueError:p.error('Formato --stages: 1:100,2:350,6:650')
    if not stages or any(n not in (1,2,6) or e<1 for n,e in stages):p.error('Etapas invalidas')
    torch.set_num_threads(a.torch_threads)
    random.seed(a.seed); np.random.seed(a.seed); torch.manual_seed(a.seed)
    out=Path(a.out).resolve();out.mkdir(parents=True,exist_ok=True)
    latest=out/'checkpoint_latest.pt'
    if latest.exists() and not a.resume:raise SystemExit('La salida ya tiene checkpoint: usa --resume o cambia --out')
    cfg=PPOConfig(learning_rate=a.learning_rate)
    model=MAPPO(); optimizer=torch.optim.Adam(model.parameters(),lr=cfg.learning_rate)
    if a.resume:
        if not latest.exists():raise SystemExit('No existe checkpoint para --resume')
        state=torch.load(latest,map_location='cpu',weights_only=False)
        if state['seed']!=a.seed:raise SystemExit('Semilla distinta de checkpoint')
        model.load_state_dict(state['model']);optimizer.load_state_dict(state['optimizer'])
        stage_start,episode_start=state['stage'],state['episode']
    else:
        source=Path(a.source).resolve()
        if not source.exists():raise SystemExit(f'No existe el checkpoint origen: {source}')
        state=torch.load(source,map_location='cpu',weights_only=False)
        model.load_state_dict(state['model'],strict=True)
        # Transferencia de pesos, NO continuacion exacta del optimizador anterior.
        stage_start=episode_start=0
    config={'source':str(a.source),'seed':a.seed,'stages':stages,'learning_rate':a.learning_rate,
            'eval_every':a.eval_every,'eval_episodes':a.eval_episodes,
            'final_eval_episodes':a.final_eval_episodes,'ik_stride':os.getenv('SWARM_IK_STRIDE','1'),
            'note':'Se transfieren pesos actor y critico; optimizador nuevo; evaluacion determinista con semillas separadas.'}
    confpath=out/'configuration_final.json'
    if a.resume and confpath.exists():
        old=json.loads(confpath.read_text())
        for key in ('seed','stages','learning_rate'):
            if old[key]!=config[key]:raise SystemExit(f'Configuracion de resume incompatible: {key}')
    else:confpath.write_text(json.dumps(config,indent=2))
    deadline=time.monotonic()+a.hours*3600
    best_by_stage={}
    bestfile=out/'best_by_stage.json'
    if bestfile.exists():best_by_stage=json.loads(bestfile.read_text())
    for stage_idx,(agents,n) in enumerate(stages):
        if stage_idx<stage_start:continue
        if time.monotonic()>=deadline:break
        learning_cfg=LearningConfig(goal_radius_m=0.04)
        env=SwarmLearningEnv(num_agents=agents,platform_rotation=False,seed=a.seed,gui=False,config=learning_cfg)
        evalenv=SwarmLearningEnv(num_agents=agents,platform_rotation=False,seed=a.seed+100000,gui=False,config=learning_cfg)
        try:
            begin=episode_start if stage_idx==stage_start else 0
            for e in range(begin,n):
                if time.monotonic()>=deadline:break
                _,metrics=(),None
                records,metrics=episode(env,model,a.seed+stage_idx*100000+e,train=True)
                metrics.update(update(model,optimizer,records,agents,cfg))
                append_csv(out/'training.csv',dict(phase='training',stage=stage_idx+1,episode=e+1,agents=agents,**metrics))
                next_stage,next_episode=(stage_idx,e+1) if e+1<n else (stage_idx+1,0)
                save(latest,model,optimizer,next_stage,next_episode,a.seed,cfg)
                if (e+1)%a.eval_every==0 or e+1==n:
                    # Semillas de evaluacion fijas dentro de cada etapa para comparar checkpoints.
                    seeds=[a.seed+500000+stage_idx*10000+j for j in range(a.eval_episodes)]
                    summary=evaluate(evalenv,model,seeds)
                    append_csv(out/'evaluation.csv',dict(phase='evaluation',stage=stage_idx+1,episode=e+1,agents=agents,eval_count=a.eval_episodes,**summary))
                    key=str(agents)
                    score=[summary['team_success'], -summary['invalid_actions'], -summary['final_distance_mean_m']]
                    old=best_by_stage.get(key)
                    if old is None or score>old['score']:
                        path=out/f'best_{agents}arms.pt'
                        save(path,model,optimizer,next_stage,next_episode,a.seed,cfg)
                        best_by_stage[key]={'score':score,'episode':e+1,'stage':stage_idx+1,'metrics':summary,'path':str(path)}
                        bestfile.write_text(json.dumps(best_by_stage,indent=2))
                    print('EVALUATION',agents,'arms',e+1,summary,flush=True)
            if time.monotonic()>=deadline:break
        finally:
            env.close();evalenv.close()
    # Validacion independiente, con semillas diferentes de las evaluaciones periodicas.
    for key,record in best_by_stage.items():
        if time.monotonic()>=deadline:break
        agents=int(key); state=torch.load(record['path'],map_location='cpu',weights_only=False)
        model.load_state_dict(state['model'])
        env=SwarmLearningEnv(num_agents=agents,platform_rotation=False,seed=a.seed+800000,gui=False,config=LearningConfig(goal_radius_m=0.04))
        try:
            seeds=[a.seed+800000+agents*10000+j for j in range(a.final_eval_episodes)]
            result=evaluate(env,model,seeds)
            (out/f'validation_{agents}arms.json').write_text(json.dumps({'episodes':a.final_eval_episodes,'metrics':result,'checkpoint':record['path']},indent=2))
            print('FINAL_VALIDATION',agents,result,flush=True)
        finally:env.close()
    print('FINALIZADO. Resultados en',out,flush=True)

if __name__=='__main__':main()

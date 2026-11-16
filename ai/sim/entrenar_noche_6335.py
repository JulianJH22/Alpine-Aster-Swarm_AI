#!/usr/bin/env python3
"""Orquestador nocturno: BC, evaluacion fisica y MAPPO con presupuesto temporal."""
import argparse
import csv
import datetime as dt
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent

def run_command(cmd, log, deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        return 'sin_tiempo'
    print('\nEJECUTANDO:', ' '.join(map(str,cmd)), flush=True)
    with log.open('a', encoding='utf-8') as f:
        f.write('\n=== '+dt.datetime.now().isoformat()+' ===\n'+' '.join(map(str,cmd))+'\n')
        f.flush()
        proc = subprocess.Popen(cmd, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT,
                                env={**os.environ, 'SWARM_IK_STRIDE':os.environ.get('SWARM_IK_STRIDE','4')})
        try:
            return proc.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try: proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait()
            return 'tiempo_agotado'

def summarize(out):
    summary = {'generated_at':dt.datetime.now().isoformat()}
    bc = out/'bc'/'report.json'
    if bc.exists():
        data=json.loads(bc.read_text()); summary['bc']={k:data.get(k) for k in ['samples','best_validation_mse','elapsed_s']}
    for agents in (1,2,6):
        p=out/f'evaluation_{agents}arms.json'
        if p.exists():
            data=json.loads(p.read_text());summary[f'evaluation_{agents}arms']={k:{m:v.get(m) for m in ['team_success_rate','final_distance_mean_m','invalid_mean','steps_mean']} for k,v in data.items()}
    for kind in ('training','evaluation'):
        p=out/'mappo'/f'{kind}.csv'
        if p.exists():
            with p.open(newline='') as f:
                rows=list(csv.DictReader(f))
            summary[kind]={'rows':len(rows),'last_5':rows[-5:]}
    (out/'resumen.json').write_text(json.dumps(summary,indent=2))
    return summary

def main():
    p=argparse.ArgumentParser(description='Entrenamiento nocturno de IA Enjambre')
    p.add_argument('--hours',type=float,default=8)
    p.add_argument('--out',default='out/noche6335')
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--samples',type=int,default=30000)
    p.add_argument('--eval-episodes',type=int,default=8)
    p.add_argument('--skip-bc',action='store_true')
    args=p.parse_args()
    if args.hours<=0 or args.samples<100 or args.eval_episodes<1: p.error('Parametros invalidos')
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=True)
    deadline=time.monotonic()+args.hours*3600
    log=out/'noche.log'
    results={}
    required=['train_mappo_63.py','pretrain_bc_6334.py','evaluate_hybrid_6334.py']
    missing=[f for f in required if not (ROOT/f).exists()]
    if missing: raise SystemExit('Faltan scripts en sim: '+', '.join(missing)+'\nExtrae el ZIP completo de Fase 6.3.3.5.')
    bc=out/'bc'/'bc_actor_best.pt'
    if not args.skip_bc and not bc.exists():
        anchors=ROOT/'out/bc6333/demonstrations.npz'
        cmd=[sys.executable,'-u','pretrain_bc_6334.py','--out',str(out/'bc'), '--samples',str(args.samples),'--epochs','100','--patience','15','--seed',str(args.seed)]
        if anchors.exists():cmd+=['--reuse-anchors',str(anchors)]
        else:cmd+=['--episodes','60']
        results['bc']=run_command(cmd,log,deadline)
    if bc.exists():
        for agents in (1,2,6):
            target=out/f'evaluation_{agents}arms.json'
            if target.exists():continue
            cmd=[sys.executable,'-u','evaluate_hybrid_6334.py','--checkpoint',str(bc),'--agents',str(agents),'--episodes',str(args.eval_episodes),'--seed',str(1000+agents),'--out',str(target)]
            results[f'eval_{agents}']=run_command(cmd,log,deadline)
            summarize(out)
            if results[f'eval_{agents}'] == 'tiempo_agotado':break
    # Se usa BC para inicializar MAPPO solo si supera un umbral en evaluacion de 1 brazo.
    bc_ok=False
    e=out/'evaluation_1arms.json'
    if bc.exists() and e.exists():
        r=json.loads(e.read_text());bc_ok=r.get('bc_actor',{}).get('team_success_rate',0)>=0.25
    mappo=out/'mappo';mappo.mkdir(exist_ok=True)
    checkpoint=mappo/'checkpoint_latest.pt'
    if deadline-time.monotonic()>30:
        cmd=[sys.executable,'-u','train_mappo_63.py','--out',str(mappo),'--seed',str(args.seed),'--eval-every','25','--eval-episodes','3']
        if checkpoint.exists():cmd+=['--resume']
        elif bc_ok:cmd+=['--bc-actor',str(bc)]
        results['mappo']=run_command(cmd,log,deadline)
    summary=summarize(out)
    summary['process_results']=results
    summary['bc_initialized_mappo']=bc_ok
    (out/'resumen.json').write_text(json.dumps(summary,indent=2))
    print('\nFINALIZADO. Revisa:',out/'resumen.json','y',log,flush=True)
    print('El checkpoint de MAPPO se guarda despues de cada episodio.',flush=True)

if __name__=='__main__':main()

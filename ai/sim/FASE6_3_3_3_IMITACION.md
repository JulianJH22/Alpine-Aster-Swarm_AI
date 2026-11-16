# Fase 6.3.3.3 — Behavior Cloning experimental

El profesor es geométrico: emite una acción TCP hacia el objetivo y deja que el entorno aplique sus validaciones. No es un controlador certificado de seguridad ni garantiza ausencia de colisiones entre eslabones. El conjunto supervisado contiene únicamente observaciones en las que el brazo está libre para aceptar una acción.

## Uso rápido (desde `sim`)

```bash
export SWARM_IK_STRIDE=4
python pretrain_bc_6333.py --agents 1 --episodes 40 --epochs 20 --out out/bc6333
python train_mappo_63.py --bc-actor out/bc6333/bc_actor_best.pt --episodes 60 --eval-every 20 --eval-episodes 3 --out out/mappo_bc6333
```

Para comparar contra el entrenamiento sin BC:

```bash
python train_mappo_63.py --episodes 60 --eval-every 20 --eval-episodes 3 --out out/mappo_baseline6333
```

Nota: `--episodes 60` recorre el currículo existente en orden y por tanto solo entrena un brazo. Para evaluar seis brazos se necesita completar las etapas anteriores o modificar explícitamente el currículo en una futura versión. El profesor actual no es multiagente avanzado y la calidad de sus demostraciones debe comprobarse en `report.json` antes de usar sus pesos. La evaluación física debe repetirse con `SWARM_IK_STRIDE=1` antes de validar el resultado final.

Los checkpoints `bc_actor_best.pt` contienen únicamente los pesos del actor. El crítico se inicializa de cero; `--resume` conserva la prioridad sobre `--bc-actor`. Los archivos de la fase anterior se mantienen salvo la adición del argumento opcional al entrenador.

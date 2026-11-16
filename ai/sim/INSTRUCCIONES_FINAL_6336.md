# Fase 6.3.3.6 — Entrenamiento final

No ejecutar simultaneamente con el entrenamiento 6335: compiten por CPU.

1. Esperar a que termine el proceso nocturno anterior. **No copiar un checkpoint mientras esta siendo escrito**. Preferir `checkpoint_latest.pt` cuando el proceso haya terminado.
2. Copiar `entrenar_final_6336.py` a la carpeta `sim` del proyecto WSL (si ya se descomprimio el ZIP completo, ya estara).
3. Desde `sim` y con `.venv` activado:

```bash
export SWARM_IK_STRIDE=4
nohup python -u entrenar_final_6336.py \
  --source out/noche6335/mappo/checkpoint_latest.pt \
  --out out/final6336 --hours 8 \
  --stages 1:100,2:350,6:650 \
  --eval-every 25 --eval-episodes 12 --final-eval-episodes 30 \
  > final6336.log 2>&1 < /dev/null &
```

Monitorizar: `tail -f final6336.log`. Consultar `out/final6336/evaluation.csv` y `out/final6336/best_by_stage.json`.

Para reanudar una ejecucion interrumpida con la misma configuracion, agregar `--resume` al comando. El limite de horas empieza de nuevo. No usar la misma carpeta de salida para ejecuciones independientes.

El origen se carga como inicializacion de pesos actor y critico, no como continuacion exacta de Adam. El critico del checkpoint inicial puede estar entrenado para otra etapa; su rendimiento debe validarse. No se garantiza convergencia ni exito de seis brazos. La plataforma sigue fija.

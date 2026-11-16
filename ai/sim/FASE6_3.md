# FASE 6.3 — Entrenamiento progresivo MAPPO

La politica **actor es compartida**: cada brazo ejecuta inferencia a partir de su observacion local, con los mismos pesos. El critico usa un estado global de longitud fija con mascara, solo durante entrenamiento. No se ha implementado aun despliegue en procesos/dispositivos separados ni comunicacion entre vecinos.

## Uso

Desde `sim/`, con PyBullet, numpy y torch instalados:

```bash
python train_mappo_63.py --calibrate --out out/phase63_calibration
python train_mappo_63.py --episodes 2 --eval-every 1 --eval-episodes 2 --out out/phase63_smoke
python train_mappo_63.py --out out/phase63_full
```

`--resume --out out/phase63_full` reanuda desde `checkpoint_latest.pt` con la misma semilla. No ejecutar dos procesos sobre el mismo directorio de salida. La calibracion son 30 episodios adicionales y NO inicia automaticamente el curriculum de 1000.

Los 1000 episodios estan divididos en 300 (1 brazo), 300 (2) y 400 (6), sin rotacion de plataforma. Cada 25 episodios se realizan 10 evaluaciones deterministas con semillas separadas. Las evaluaciones NO actualizan pesos. La evaluacion final de cada etapa tambien guarda checkpoint.

Los archivos `training.csv` y `evaluation.csv` registran recompensa global, recompensa por agente, exitos, exito del equipo, distancias finales, acciones invalidas y desglose de recompensa. `configuration.json` fija la configuracion de la corrida.

## Precauciones

- Se corrigio la bonificacion de exito repetida: se concede una sola vez por agente y episodio.
- La recompensa del equipo se calcula sumando las individuales. Por ello es fundamental comparar tambien `reward_per_agent`.
- El preflight de PyBullet sigue sin verificar IK completa ni todas las colisiones de eslabones. No se debe interpretar ausencia de movimientos invalidos como seguridad fisica.
- El evaluador no considera colisiones que el entorno no expone. No se han implementado aun normalizacion de retornos ni parada automatica por divergencia.
- Los checkpoints de cada etapa incluyen pesos, optimizador, semilla y progreso. Reanudar requiere la misma configuracion de etapas.
- `--episodes` es para pruebas de humo, no para modificar un entrenamiento ya empezado.

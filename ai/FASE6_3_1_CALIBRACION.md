# FASE 6.3.1 — Diagnóstico y recalibración

Esta versión **no ejecuta** los 1.000 episodios ni altera la arquitectura MAPPO: actor compartido, crítico centralizado, plataforma fija.

## Cambios

- Objetivos de calibración próximos (radio de 2,5 cm); el entrenamiento completo mantiene radio de 4 cm.
- Ganancia de progreso de 10 a 40, sin modificar la bonificación de éxito (una vez por brazo).
- Métricas nuevas: distancia inicial, reducción absoluta, reducción porcentual, radio de objetivos y número de evaluaciones.
- `evaluation_detail.csv`: una fila por cada episodio de evaluación, además de los promedios de `evaluation.csv`.
- Evita sobrescribir por accidente `training.csv` si la carpeta ya contiene datos y no se usa `--resume`.
- Diagnóstico determinista sin aprendizaje para comparar el controlador orientado al objetivo con acciones nulas.

## Procedimiento

Desde `sim/` y con el entorno virtual activo:

```bash
python diagnostico_631.py --seeds 5
python train_mappo_63.py --calibrate --out out/phase631_calibration
```

**Importante:** usa una carpeta nueva, no `out/phase63_calibration`, para preservar los datos anteriores.

Revisa `out/phase631_diagnostic.json`, `training.csv`, `evaluation.csv` y `evaluation_detail.csv`. Si el controlador dirigido al objetivo no reduce consistentemente la distancia, revisa el controlador PyBullet/IK antes de aumentar los episodios. Un resultado positivo del diagnóstico no garantiza que MAPPO converja.

## Limitaciones

La viabilidad del movimiento continúa basada en preflight TCP, no en una comprobación completa de colisiones de eslabones. No se ha modificado el control de las articulaciones ni se han entrenado modelos nuevos en esta entrega. Las recompensas y el tamaño de objetivo son hipótesis para recalibración, no parámetros optimizados.

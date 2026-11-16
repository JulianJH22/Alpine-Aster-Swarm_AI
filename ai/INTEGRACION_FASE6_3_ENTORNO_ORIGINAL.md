# FASE 6.3 integrada sobre el entorno industrial original

**Fuente de verdad visual y física:** `IA_Enjambre_Fase5_6_entorno_industrial_v2(2).zip`.
Se conservaron byte a byte `sim/swarm_sim/world.py`, `sim/swarm_sim/arm.py`,
`sim/run_demo.py` y `sim/config/scenario.json`. No se usaron las geometrías
experimentales de 36/42/56 cm de los intentos posteriores.

Se incorporaron desde `IA_Enjambre_Fase6_3_1_Correccion_Calibracion(1).zip`
`sim/learning/`, `train_mappo_63.py`, `train_mappo_62.py`,
`diagnostico_631.py`, configuración y pruebas del aprendizaje. El script
`train_mappo_63.py` añade solamente la opción `--gui` para visualizar
el mismo mundo durante entrenamiento; por defecto usa DIRECT.

## Instalación y comprobaciones

```bash
cd IA_Enjambre_Fase5/sim
python -m pip install -r requirements.txt
python verificar_integracion_63.py
python run_demo.py --gui
python diagnostico_631.py --seeds 5
python -m pytest tests -q
```

## Continuar entrenamiento

```bash
# Primero prueba corta (no equivale a calibración exitosa)
python train_mappo_63.py --calibrate --episodes 1 --eval-episodes 1 --out out/smoke_original
# Calibración de 30 episodios
python train_mappo_63.py --calibrate --out out/calibracion_original
# Currículo 300 + 300 + 400 episodios, una vez revisadas métricas
python train_mappo_63.py --out out/entrenamiento_original
# Para ver el escenario durante la prueba, añade --gui
```

**Advertencia de geometría heredada:** la fuente original monta los brazos
a 0,68 m del centro, aunque el alcance nominal declarado es 0,43 m desde
el hombro. No se garantiza el acceso al centro ni la convergencia de MAPPO.
El entrenamiento 6.3 entrena movimientos TCP próximos a la postura inicial,
no la clasificación industrial completa. Los elementos industriales añadidos
por el original son visuales y no realizan transporte físico automáticamente.

La igualdad del escenario NO significa que la mecánica sea viable; esta
integración deliberadamente no altera el entorno de referencia.

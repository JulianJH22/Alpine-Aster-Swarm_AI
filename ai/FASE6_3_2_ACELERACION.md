# Fase 6.3.2 — iteración rápida

Se conserva sin cambios el entorno industrial original (`world.py`, `arm.py`, `run_demo.py`, `scenario.json`).

Cambios: evaluaciones cada 50 episodios y 5 episodios por evaluación por defecto, en vez de 25 y 10; un hilo de PyTorch por defecto para redes pequeñas. Los checkpoints se mantienen.

Desde `sim/`:

    python train_mappo_63.py --calibrate --out out/calibracion_632
    python train_mappo_63.py --episodes 100 --eval-every 25 --eval-episodes 5 --out out/piloto_632

Esto reduce sobrecoste, pero no garantiza mejora del aprendizaje. No se ha ejecutado PyBullet en este entorno. No reanudar checkpoints antiguos sin revisar compatibilidad.

# FASE 5.5 — Registro, métricas y reproducción

Esta entrega **no entrena MAPPO**. El entrenamiento empieza en la FASE 6.

## Uso

Terminal 1:
```bash
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate
cd ~/IA_Enjambre_Fase5/sim
python -u run_demo.py --gui --serve 7777
```

Terminal 2:
```bash
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate
cd ~/IA_Enjambre_Fase5/sim
python -m pip install matplotlib
python -u control_tareas_54.py
```

En `enjambre>`:
```text
test1
guardar
graficas
```

Los resultados se guardan en `sim/out/phase55/ultimo/`: `episode.json`, `metrics.csv`, `summary.json`, `altura_tcp.png` y `velocidad_articular.png`.

Para reproducir en PyBullet, cierra la consola y el simulador servidor (Ctrl+C) y ejecuta:
```bash
cd ~/IA_Enjambre_Fase5/sim
python reproducir_55.py
```

## Limitaciones explícitas

- La reproducción muestra las articulaciones, posiciones de objetos y ángulo de plataforma **registrados**. No recalcula contactos, pinzas ni dinámica: es una visualización cinemática aproximada.
- La orientación de los objetos no está registrada, y se conserva la orientación inicial en la reproducción.
- Las gráficas iniciales muestran altura TCP y velocidad articular. **No son gráficas de error MPC**: para eso hay que registrar objetivos y predicciones sincronizados con la ejecución (pendiente).
- La grabación se extrae del búfer de telemetría del servidor. Un `reset` elimina el historial anterior: ejecutar `guardar` antes de reiniciar.
- Los registros de entrenamiento/recompensa por episodio no existen hasta la FASE 6.

# FASE 5.3 — Demostración visual (antes de 5.4)

Esta demostración utiliza **los seis brazos**, uno tras otro, y hace que cada brazo
intente elevar su herramienta 2 cm y volver a su posición inicial. Los comandos
se validan con el preflight de la FASE 5.3 y se ejecutan con el controlador
PyBullet existente. Se registran los resultados, incluidos fallos y rechazos.

## Ubuntu

```bash
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate
cd ~/IA_Enjambre_Fase5/sim
python -u demo_visual_53.py --gui
```

No abras `run_demo.py --serve 7777` para esta prueba: el nuevo script crea su
propia instancia local del simulador y no necesita el servidor TCP.

Al terminar, pulsa Enter en la terminal para cerrar la ventana. Los resultados
quedan en `sim/out/demo_visual_53.json`.

## Qué deberías ver

Seis brazos alrededor de la plataforma. Cada brazo intenta una elevación
vertical corta y vuelve al inicio, **secuencialmente**. Los rechazos de alcance
o fallos de cinemática se muestran en la terminal y en el JSON.

## Límites de la demostración

- No entrena MAPPO (reservado para FASE 6).
- El planificador actual es un preflight TCP; **no es un MPC optimizado**.
- No demuestra movimientos simultáneos ni garantiza ausencia de colisiones
  entre eslabones. La seguridad de robots reales NO está validada.
- El Panda URDF escalado a 43 cm es provisional, no el robot físico definitivo.
- La ejecución en PyBullet GUI debe comprobarse en tu equipo Ubuntu; aquí solo
  se verificó la sintaxis del script, no la dinámica física.

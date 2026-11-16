# FASE 5.4 — Corrección de espera y pruebas concatenadas

## Corrección
El cliente TCP anterior conservaba un timeout de 10 segundos después de conectar. Un reinicio o una ejecución larga podía provocar una excepción no capturada y cerrar la consola. Ahora se configura una espera de respuesta de 180 segundos y se muestra un error recuperable. No se ha cambiado el timeout de tiempo simulado ni la física de PyBullet.

## Pruebas desde `enjambre>`
- `test1`: brazos 1–2, cada uno sube 6 cm desde su posición medida y vuelve.
- `test2`: brazos 1–4, misma secuencia.
- `test3`: brazos 1–6, misma secuencia.

Los movimientos son **secuenciales**. Cada acción realiza preflight y consulta el resultado de ejecución; si falla, se detiene la secuencia. Se avanza el simulador en bloques de 0,15 segundos con pequeñas pausas para facilitar la observación. **No hay agarre ni entrenamiento.**

## Ejecución
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
python -u control_tareas_54.py
```
Dentro de `enjambre>`: `test1`, luego `test2` o `test3`.

Si `robot_busy`, ejecuta `terminar 30` y comprueba `estado`; si el movimiento es rechazado, se muestra el motivo. Un plan aceptado no garantiza la ejecución física. Para reiniciar, `reiniciar` y confirmar `s`.

## Pruebas automáticas
Desde la carpeta principal: `python -m pytest -q sim/tests/test_control_tareas_phase54.py sim/tests/test_macros_phase54.py`

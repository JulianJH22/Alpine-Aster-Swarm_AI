# FASE 5.4 — Control interactivo de tareas (terminal)

## Objetivo
Controlar de manera interactiva el simulador PyBullet y el módulo lógico de residuos sin editar JSON ni recompilar C++.

## Terminal 1 — servidor PyBullet
```bash
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate
cd ~/IA_Enjambre_Fase5/sim
python -u run_demo.py --gui --serve 7777
```

## Terminal 2 — consola de tareas
```bash
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate
cd ~/IA_Enjambre_Fase5/sim
python control_tareas_54.py
```
Escribe `ayuda` para ver todos los comandos. Ejemplos:
```
estado
plan arm_1 0.25 0.00 0.25
mover arm_1 0.25 0.00 0.25
terminar 30
inspeccionar assembly_board
desmontar assembly_board clip_tool
probar assembly_board
clasificar assembly_board electronico no_funciona
resumen
reiniciar
```
**Aviso:** las coordenadas de ejemplo no garantizan alcance; comprueba `plan` antes de mover. El preflight es aproximado y el simulador conserva las validaciones físicas. `depositar` y `agarrar` son órdenes físicas que pueden fallar si el objetivo es inalcanzable o no existe un agarre válido.

## Alcance y limitaciones
- Control interactivo de tareas por terminal. El panel web se desarrollará en una entrega posterior.
- `mover` realiza preflight antes de enviar; `depositar` utiliza el validador existente del simulador.
- La lógica de reciclaje sigue siendo **lógica**; no representa desmontaje físico.
- El tiempo de simulación avanza mediante `avanzar` o `terminar`; no basta con enviar una orden.
- Sin MAPPO ni entrenamiento: ambos reservados para FASE 6.

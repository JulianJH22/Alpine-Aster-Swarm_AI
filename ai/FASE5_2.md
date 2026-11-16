# FASE 5.2 — Residuos y desmontaje progresivo

## Alcance implementado

- Modelo lógico de componentes con materiales `plastico`, `madera`, `electronico`.
- Estados funcionales `funciona`, `no_funciona`, `sin_determinar`.
- Inspección que no revela el estado funcional real; prueba funcional simulada ideal.
- Uniones lógicas `clip` (herramienta `clip_tool`) y `screw` (herramienta `screwdriver`).
- Clasificación con verificación interna y rechazo de clasificación antes de probar.
- Reinicio del modelo lógico con el `reset` ya existente de FASE 5.1.
- Gateway TCP/JSON con nuevas operaciones, sin romper las existentes.
- Cinco pruebas unitarias independientes de PyBullet.

## Comandos de prueba del gateway

Arrancar el servidor de la forma habitual. En otra terminal, con Python 3:

```python
from swarm_sim.gateway import GatewayClient
c = GatewayClient('127.0.0.1', 7777)
print(c.call('recycling_inspect', component_id='assembly_board'))
print(c.call('recycling_disassemble', component_id='assembly_board', tool='clip_tool'))
print(c.call('recycling_test', component_id='assembly_board'))
print(c.call('recycling_classify', component_id='assembly_board', material='electronico', functional='no_funciona'))
print(c.call('recycling_summary'))
c.close()
```

Ejecutar desde `sim/` con el entorno virtual de PyBullet activado.

## Límites conocidos y tareas futuras

**No hay desmontaje físico todavía.** Los componentes `assembly_case`, `assembly_board` y `assembly_cover` son entidades lógicas: no se crean como cuerpos PyBullet. Las operaciones no mueven brazos, no accionan herramientas y no cambian la geometría. Los objetos `obj_1`, `obj_2` y `obj_3` sí son objetos físicos ya existentes, pero la clasificación no los transporta.

El sensor funcional es ideal y determinista. Para FASE 6 se podrá introducir ruido, fallos de sensor y observaciones físicas, sin revelar la verdad de referencia a la política.

La geometría de los seis brazos y la plataforma se conserva. No se entrena MAPPO ni se ejecutan episodios de entrenamiento.

## Pruebas

```bash
python -m pytest -q sim/tests/test_recycling_phase52.py
```

La prueba física integral de PyBullet y la integración con C++ requieren validación en Ubuntu con las dependencias instaladas.

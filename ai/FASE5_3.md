# FASE 5.3 — Preplanificacion predictiva TCP (entrega incremental)

## Alcance real

Se añade un **preplanificador predictivo conservador**, no un MPC optimo completo.
Permite consultar la factibilidad aproximada de movimientos `Move` y enviarlos
solo si superan la comprobacion preliminar. No se ha conectado aun la
manipulacion fisica de residuos ni el desmontaje a movimientos de brazos.
No existe entrenamiento MAPPO: queda reservado para FASE 6.

## Cambios respecto a 5.2

- `sim/swarm_sim/mpc_preflight.py`: prediccion de waypoints y chequeo de
  alcance esferico nominal, suelo y separacion TCP a lo largo del recorrido.
- `sim/swarm_sim/gateway.py`: operaciones `plan_move` y `submit_planned`.
- `sim/tests/test_mpc_preflight_phase53.py`: cinco pruebas de la logica.
- Este documento.

## Protocolo (TCP/JSON-lines en puerto 7777)

Consulta que NO mueve el brazo:

```json
{"id":1,"op":"plan_move","args":{"robot_id":"arm_1","target":[0.2,0.0,0.2],"max_speed":0.1}}
```

`feasible_preflight=false` implica rechazo del plan. `true` NO garantiza que
la cinemática inversa converja ni que el brazo completo no colisione.

Envio de movimiento condicionado (SI mueve el brazo al avanzar la simulacion):

```json
{"id":2,"op":"submit_planned","args":{"command":{"robot_id":"arm_1","execution_id":"prueba_unica_001","type":"Move","target":{"position":[0.2,0.0,0.2],"frame":"world"},"max_speed":0.1,"tolerance":0.01}}}
```

Tras la aceptacion hay que avanzar el tiempo con `step` o `run_until_idle`
y consultar `status` o `snapshot`. Se mantienen los mecanismos originales de
seguridad de `SimRobotArm`.

## Pruebas

Desde la raiz del proyecto, con `pytest` instalado:

```bash
python -m pytest -q sim/tests/test_recycling_phase52.py sim/tests/test_mpc_preflight_phase53.py
cmake -S . -B build && cmake --build build -j && ctest --test-dir build --output-on-failure
```

## Limitaciones y siguientes trabajos de FASE 5.3

1. La comprobacion de colision solo considera puntos TCP y otros brazos en
   posiciones actuales; no geometria completa ni trayectorias simultaneas.
2. El control real continua siendo el servocontrol con IK de la FASE 4.
   No se ha implementado un MPC optimizador con funcion de coste y restricciones.
3. La validacion PyBullet extremo a extremo requiere un entorno con PyBullet.
4. El modelo Franka escalado a 43 cm es provisional y no certifica alcance real.
5. Antes de afirmar completada la FASE 5.3, faltan un MPC de horizonte
   recedente integrado, chequeos de colision volumetricos, reservas de zonas
   y pruebas fisicas multi-brazo.

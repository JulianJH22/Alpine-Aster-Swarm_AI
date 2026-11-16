# Gemelo digital IA Enjambre — FASE 4 (v0.1)

Simulador 3D (PyBullet) de **6 brazos en los vértices de un hexágono** que implementa los contratos del
documento de traspaso (`IRobotArm`, `IEnvironment`) con maniobras **asíncronas**, telemetría con tiempo de
simulación, detección de colisiones/límites e inyección de fallos. Todo es **determinista** y
**parametrizable por `config/scenario.json`**: la geometría no está en el código.

> Estado honesto: es la base de FASE 4, no un gemelo validado. Los valores de `scenario.json` son
> provisionales y la equivalencia con el robot real exige calibración (ver «Limitaciones»).

## Arranque

```bash
pip install -r requirements.txt
python -m pytest tests -q            # 23 pruebas (~20 s)
python run_demo.py --gui --realtime  # ventana 3D (necesita pantalla)
python run_demo.py --render out/     # fotos PNG sin pantalla + out/telemetry.jsonl
python run_demo.py --serve 7777      # pasarela TCP para el núcleo C++
```

## Estructura

| Archivo | Contenido |
|---|---|
| `config/scenario.json` | Hexágono, modelos de brazo, estaciones, seguridad, zonas, objetos, física, telemetría |
| `swarm_sim/types.py` | Tipos y protocolos (equivalentes ampliados de `core.hpp`), SI + cuaternión |
| `swarm_sim/arm.py` | `SimRobotArm`: IK + control de motores, maniobras, validación, fallos |
| `swarm_sim/world.py` | `SimWorld`, `SimEnvironment`, objetos, zonas, monitor de seguridad, render |
| `swarm_sim/telemetry.py` | Eventos, muestreo periódico, `digest()` para determinismo, volcado JSONL |
| `swarm_sim/gateway.py` | Pasarela TCP + JSON-lines (+ cliente de referencia) |
| `tests/test_scenarios.py` | Escenarios (a)–(f) de la sección 7 y criterios de la sección 10 |

## Mapeo con el contrato C++ v0.2

| C++ (`core.hpp`) | Aquí | Cambio |
|---|---|---|
| `execute()` | `execute()` (bloqueante) **y** `submit()` + `result()` | `submit` es el flujo asíncrono objetivo |
| `ExecutionStatus {Accepted,Rejected,Completed,Failed}` | + `running`, `cancelled` | ciclo de vida de la sección 5 |
| `Pose` (Euler) | `Pose` (posición + cuaternión xyzw, `frame`) | `from_dict` acepta el formato Euler antiguo; ángulos a 0 ⇒ «sin orientación» |
| `RobotState` | + `sim_time`, `valid`, articulaciones, TCP, pinza, objeto sujeto | |
| `ManeuverCommand` | + `execution_id`, `robot_id`, `max_speed`, `tolerance`, `hold_time`, `timeout_s` | |

Convenios (todavía por aprobar): metros, radianes, segundos, SI; solo `frame="world"` (otro se rechaza con
`unsupported_frame`); IDs `arm_1..arm_6`; un `execution_id` único por maniobra.

## Semántica de maniobras implementada (a validar con los controladores reales)

| Maniobra | Qué hace | Falla / se rechaza con |
|---|---|---|
| `Move` | TCP en línea recta al objetivo (herramienta hacia abajo si no hay orientación) | `target_not_reached` si no llega en tolerancia |
| `Inspect` | Se coloca 0.25 m sobre el objeto; devuelve su observación en `data` | |
| `Grasp` | Abre, aproxima, desciende, verifica distancia, cierra, sujeta, eleva | `grasp_failed`, `payload_exceeded`, `object_too_wide_for_gripper`, `gripper_occupied` |
| `Hold` | Mantiene pose y agarre (`hold_time` o hasta `cancel`) | |
| `Release` | Suelta el objeto y abre la pinza | `not_holding_object` |
| `Sort` | Lleva el objeto sujeto al objetivo, lo suelta y se retira | `not_holding_object` |

Rechazos previos a ejecutar: `busy`, `out_of_reach`, `below_floor`, `unknown_object`,
`predicted_collision_with: arm_X` (objetivo a menos de `min_tcp_separation_m` de otro brazo o de su objetivo),
`comm_lost`, `duplicate_execution_id`, `unsupported_maneuver`.
Durante la ejecución: `collision` (contacto físico entre brazos), `separation_violation` (TCPs demasiado cerca
mientras alguno se mueve), `timeout`. Un fallo **nunca se reporta como éxito**.

## Pasarela para el núcleo C++ (pregunta 2 del documento)

Una línea JSON por petición/respuesta sobre TCP. **El tiempo lo controla el cliente** (`step`), por lo que
las ejecuciones son reproducibles y el MPC puede sincronizarse con `sim_time`.

```json
-> {"id":1,"op":"submit","args":{"command":{"execution_id":"n1","robot_id":"arm_1","type":"Move",
     "target":{"x":0.15,"y":0.0,"z":0.25,"roll":0,"pitch":0,"yaw":0}}}}
<- {"id":1,"ok":true,"result":{"execution_id":"n1","status":"accepted", ...}}
-> {"id":2,"op":"step","args":{"seconds":0.5}}
-> {"id":3,"op":"status","args":{"robot_id":"arm_1","execution_id":"n1"}}
```

Operaciones: `hello`, `time`, `states`, `state`, `capabilities`, `observe`, `submit`, `status`, `cancel`, `step`,
`run_until_idle`, `reset`, `telemetry`, `inject_comm_loss`. El adaptador C++ (`SimRobotArm : IRobotArm`) solo
traduce `execute()` → `submit` + `run_until_idle`/`step` + `status`, y `state()` → `state`.

## Cómo adaptarlo

* **Otras dimensiones**: `hexagon.circumradius_m` (con 0.5 m los codos de brazos vecinos casi se tocan; 0.6 m deja ~10 cm).
* **Otro brazo**: añade una entrada en `robot_models` (URDF, `ee_link`, juntas, alcance, carga) y úsala en `stations[].model`. Pueden ser modelos distintos.
* **Objetos/zonas**: listas `objects` (box/cylinder/sphere) y `zones`.
* **Fallos**: `world.inject_object_displacement(...)`, `world.inject_comm_loss(...)`.

## Limitaciones conocidas

1. El agarre fija el objeto al TCP con una restricción rígida tras verificar distancia, peso y anchura; no modela fricción ni deslizamiento de la pinza.
2. La IK es de PyBullet con orientación fija «herramienta abajo»; no hay planificador de caminos: el trayecto es una recta y los obstáculos se vigilan, no se esquivan.
3. No hay autocolisión de cada brazo ni colisión brazo–suelo/objetos como causa de fallo; solo brazo–brazo.
4. `reach_m`, `payload_kg`, límites y ganancias son los del Panda de PyBullet: **no calibrados** con los brazos reales.
5. Sin sensores simulados (cámaras/fuerza). Las observaciones son el estado exacto del motor.
6. No es una certificación de seguridad industrial ni una validación cuantitativa del gemelo.
7. El tiempo de simulación no está vinculado al reloj real salvo con `--realtime`.

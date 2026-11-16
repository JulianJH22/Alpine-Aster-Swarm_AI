# Cierre de FASE 3 — Integración C++ / gemelo digital (v0.3)

## Decisiones
- Seis agentes y seis brazos, uno por vértice de la estructura hexagonal.
- El ordenador central coordina maniobras de alto nivel y roles dinámicos.
- FASE 4: entorno 3D completo, física, geometría, sensores, planificación de colisiones y validación de fidelidad.
- FASE 5: MPC. `IMpcPlanner` es solo un contrato abstracto, **no** un MPC implementado o entrenado.

## Entregado en v0.3
- `swarm_core`: se conserva la lógica C++ v0.2 y las interfaces `IRobotArm`, `IEnvironment`.
- `swarm_gateway`: adaptador POSIX TCP/JSON-lines para `sim/swarm_sim/gateway.py` (protocolo `swarm-sim/0.1`). Requiere `json-c` y `pkg-config`; se puede desactivar con `-DSWARM_GATEWAY=OFF`.
- `GatewayRobotArm`: estado, capacidades, envío no bloqueante, consulta de resultado (`poll`) y cancelación.
- `GatewayEnvironment`: lectura de observaciones válidas.
- `GatewayClient`: handshake de versión, identificadores correlativos, límites de tamaño, timeouts, avance de reloj explícito (`step`) y espera hasta inactividad (`run_until_idle`).
- Conversión Euler ZYX del núcleo C++ a cuaterniones xyzw en comandos; la orientación identidad se envía explícitamente.
- `swarm_gateway_demo`: prueba de conexión **solo lectura**, sin enviar movimientos.
- `tests/mock_gateway.py` y `tests/test_gateway_contract.py`: smoke test C++/Python sin PyBullet.

## Compilación (Linux)
```bash
sudo apt install cmake g++ pkg-config libjson-c-dev
cmake -S . -B build -DSWARM_GATEWAY=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/swarm_demo
```

## Conectar al simulador (requiere PyBullet, FASE 4)
En una terminal:
```bash
cd sim
python -m pip install -r requirements.txt
python run_demo.py --serve 7777
```
En otra terminal, desde la raíz del proyecto:
```bash
./build/swarm_gateway_demo 127.0.0.1 7777
```
El ejemplo solo consulta la configuración y los objetos; no envía órdenes.

## Semántica importante
- `GatewayRobotArm::execute()` **envía** una orden, no espera a que termine. `Accepted` no significa `Completed`.
- La física solo avanza cuando el cliente llama a `step()` o `run_until_idle()`.
- `poll(task_id)` consulta el resultado, y `cancel(task_id)` solicita cancelación.
- Los identificadores de ejecución se generan como `<robot_id>:<task_id>`; los `task_id` deben ser únicos por brazo mientras se mantenga el historial del simulador.
- El contrato C++ original usa `Pose` con Euler y solo contiene pose cartesiana; las lecturas actuales descartan orientación, articulaciones, velocidades, tiempo y metadatos adicionales. **No es un gemelo digital fiel aún**.
- `MockSafetyPolicy` siempre autoriza: solo es válida para pruebas lógicas, **nunca para actuadores reales**.
- La asignación de agentes sigue siendo rotatoria por capacidad; no implementa todavía reservas de espacio, cooperación sincronizada ni control de colisiones.
- El núcleo original libera `current_role` después de `execute()`, aunque la maniobra haya sido aceptada y continúe en el simulador. Para cooperación asíncrona real se necesita un gestor de ejecuciones activas.
- El cliente TCP actual es síncrono, de un único socket y **no thread-safe**; usarlo desde un único bucle coordinador o protegerlo externamente.
- El simulador original de la compañera se conserva sin modificar.

## Verificación ejecutada
- Compilación completa en Linux.
- `ctest`: `swarm_tests` y `gateway_contract` superadas (2/2).
- No se ejecutó la suite PyBullet ni se validaron maniobras físicas o un escenario 3D.

## Pendientes asignados a FASE 4
- Contrato de estado enriquecido y marcas temporales, frames y orientaciones.
- Ciclo de vida asíncrono con persistencia de tareas/roles y coordinación multi-brazo.
- Validación de trayectorias, zonas, límites y detección preventiva de colisiones.
- Pruebas reales contra la pasarela PyBullet y los modelos URDF específicos.
- Tratamiento de desconexiones, reintentos idempotentes, parada segura y recuperación.
- Calibración de geometría, parámetros físicos y fidelidad sim-to-real.

## Pendientes asignados a FASE 5
- Identificación/selección del modelo dinámico, horizonte, coste, restricciones, solver, sincronización y validación del MPC.

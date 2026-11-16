# IA Enjambre — Fase 3, prototipo 0.2

Núcleo experimental **C++20**, **seis agentes** y **seis brazos** en los seis vértices de una estructura hexagonal. El gemelo digital 3D de la **Fase 4** (carpeta [`sim/`](sim/README.md), PyBullet) está incluido y permanece desacoplado del núcleo mediante `IRobotArm` e `IEnvironment`.

## Compilar y probar

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/swarm_demo
```

## Gemelo digital (Fase 4, `sim/`)

Simulador 3D en Python (PyBullet) con 6 brazos en los vértices del hexágono, maniobras asíncronas, telemetría, detección de colisiones e inyección de fallos. Se configura con `sim/config/scenario.json` y expone una pasarela TCP + JSON-lines para el núcleo C++. Detalle, protocolo y limitaciones en [`sim/README.md`](sim/README.md).

```bash
cd sim
pip install -r requirements.txt
python -m pytest tests -q
python run_demo.py --render out/      # fotos PNG sin pantalla
python run_demo.py --serve 7777       # pasarela para el núcleo C++
```

**Estado de la integración:** el código de la Fase 4 está incluido tal cual, pero el adaptador C++ (`SimRobotArm : IRobotArm`, `SimEnvironment : IEnvironment`, que traducen `execute()`/`state()`/`observe()` a la pasarela) **aún no está escrito**; `main.cpp` sigue usando los mocks. Hay un test CMake opcional (`-DSWARM_SIM_TESTS=ON`) que lanza los tests Python de `sim/`.

## Actualización MPC

Se añade `IMpcPlanner` y `MultiAgentSystem::dispatchWithMpc()` como **punto de extensión**, sin elegir horizonte, modelo dinámico, función de coste ni optimizador. El planificador recibe un objetivo y una instantánea del mundo y los seis robots; propone una maniobra de alto nivel que pasa por el despacho y la política de seguridad. **No hay MPC implementado ni entrenado en esta versión**. MPC significa *Model Predictive Control* (control predictivo basado en modelos): optimiza acciones repetidamente usando un modelo de dinámica; si se desea aprendizaje, puede entrenarse o identificarse el modelo predictivo, pero no se considera al MPC un modelo entrenado por sí mismo.

## Límites actuales

- `MockRobotArm` no es simulación física: declara maniobras finalizadas instantáneamente y asigna una pose sintética.
- `MockSafetyPolicy` acepta cualquier maniobra **solo para pruebas lógicas**. **No conectarla nunca a robots físicos**.
- El reparto es *round-robin*, no coordinación MPC real. No hay control continuo, predicción de trayectorias, dinámica, coste, restricciones, sincronización, reservas ni control de colisiones.
- `IMpcPlanner` es una interfaz vacía; el test de paso directo solo verifica el contrato, **no constituye un controlador MPC**.
- No hay ROS 2, simulador 3D, hardware, telemetría real ni ejecución asíncrona.
- `config/layout.example.json` es un ejemplo: el programa C++ todavía no carga JSON. La geometría real (radio del hexágono, brazos, objetos, zonas) está en `sim/config/scenario.json`, con valores provisionales sin calibrar.

## Frontera con el gemelo digital

El gemelo digital (`sim/`) implementa los equivalentes Python de `IRobotArm` e `IEnvironment`, telemetría y ejecución virtual de maniobras. Los tipos Python amplían `core.hpp` (cuaternión, `execution_id`, estados `running`/`cancelled`, `sim_time`); ver la tabla de mapeo en `sim/README.md`. El núcleo y la interfaz MPC no dependen de un motor 3D concreto.

## Pendiente para implementación real

1. Confirmar qué se entiende por entrenamiento con MPC: identificación del modelo de dinámica, MPC convencional, MPC aprendido o combinación con aprendizaje multiagente.
2. Definir horizonte, estados, acciones, restricciones, coste y frecuencia de replanteamiento; elegir solucionador.
3. Añadir maniobras asíncronas, realimentación con timestamps y coordinación simultánea de seis brazos.
4. Implementar seguridad independiente y verificable, límites cinemáticos, reservas y control de colisiones.
5. Integrar características de los modelos de brazo existentes.
6. Escribir el adaptador C++ de la pasarela y alinear `core.hpp` con los tipos ampliados de `sim/` (ver mapeo).
7. Decidir si el sistema final es de 5 o 6 agentes: los esquemas de arquitectura hablan de A1–A5 y `SimRobotAdapter × 5`, mientras que el código (Fase 3 y `sim/`) usa 6.

## Esquemas de referencia (`docs/`)

`Esquema_general_proyecto.jpg` (arquitectura por capas), `Esquema_programacion.jpg` (diagrama de clases) y `Esquema_tareas.jpg` (secuencia de manipulación cooperativa). Nota: reflejan 5 agentes, el código actual usa 6.

## Actualización v0.3 — cierre de FASE 3

Se incorpora un adaptador opcional C++ TCP/JSON-lines al simulador de la compañera, sin modificar el simulador ni implementar MPC. Ver [FASE3_CIERRE.md](FASE3_CIERRE.md) para instrucciones, límites de seguridad, pruebas y responsabilidades de las fases 4 y 5.

# FASE 5.1 — Integración y observabilidad (sin entrenamiento)

Base: proyecto FASE 4 limpio e integrado. No se ha incorporado MAPPO ni entrenamiento.

## Cambios
- Nuevo comando `snapshot`: lectura consistente de estados medidos, objetos, plataforma y reloj.
- `reset` devuelve confirmación de estado inicial (reconstruye mundo, brazos y objetos).
- `telemetry_samples`: consulta paginada de muestras físicas, separada de eventos.
- Cliente C++ `swarm_phase5_cli` para inspección, reinicio, avance y telemetría.
- Se mantienen `swarm_control_demo`, protocolo `swarm-sim/0.1` y servidor puerto 7777.

## Ubuntu, dos terminales
Terminal 1:
```bash
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate
cd ~/IA_Enjambre_Fase5/sim
python -u run_demo.py --gui --realtime --serve 7777
```
Terminal 2:
```bash
cd ~/IA_Enjambre_Fase5
cmake -S . -B build && cmake --build build -j
ctest --test-dir build --output-on-failure
./build/swarm_phase5_cli 127.0.0.1 7777 hello
./build/swarm_phase5_cli 127.0.0.1 7777 snapshot
./build/swarm_phase5_cli 127.0.0.1 7777 step 0.5
./build/swarm_phase5_cli 127.0.0.1 7777 samples
./build/swarm_phase5_cli 127.0.0.1 7777 reset
./build/swarm_phase5_cli 127.0.0.1 7777 snapshot
```

## Limitaciones y pruebas pendientes
- No se ejecutó PyBullet en el entorno de empaquetado; verificar en Ubuntu con PyBullet instalado.
- `reset` recrea el escenario pero no garantiza bit a bit la misma trayectoria física; hay que validar determinismo con pruebas repetidas.
- El servidor original atiende una conexión a la vez. No ejecutar el controlador C++ y el cliente de diagnóstico simultáneamente.
- `snapshot` refleja las mediciones de PyBullet; no valida todavía la seguridad física ni la cinemática de los brazos de 43 cm.
- MPC, clasificación y MAPPO se reservan para entregas posteriores; el entrenamiento exclusivamente para FASE 6.

# Integracion C++ y PyBullet — Fase 4

Esta copia incorpora `swarm_control_demo` desde la version Fase 3.5.1 (IDs unicos), manteniendo el simulador Fase 4 corregido de 43 cm sin cambios adicionales.

## Terminal 1 (simulador)

```bash
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate
cd ~/IA_Enjambre_Fase4/sim
python -u run_demo.py --gui --realtime --serve 7777
```

## Terminal 2 (C++)

```bash
cd ~/IA_Enjambre_Fase4
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/swarm_control_demo 127.0.0.1 7777 config/control_demo.json
```

## Alcance y limitaciones

- El ejecutable ejecuta una demostracion de dos etapas y dos brazos (`arm_1`, `arm_4`), NO una consola interactiva ni un controlador de seis brazos.
- Las posiciones de destino de `config/control_demo.json` son provisionales, proceden de Fase 3.5 y no se han recalibrado para el escenario de 43 cm.
- No se ha probado la ejecucion fisica en PyBullet en este entorno. Los agarres, alcance y colisiones requieren validacion en Ubuntu.
- Los IDs de ejecucion son unicos en cada lanzamiento; eso no restablece el estado de los objetos.
- El ZIP original y el simulador Fase 4 se mantienen intactos.

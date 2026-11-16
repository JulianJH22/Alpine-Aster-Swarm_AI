# IA_Enjambre_Fase4 — proyecto unificado

Este proyecto reúne el núcleo C++ (C++20), la pasarela TCP/JSON, el controlador de demostración y el simulador Python/PyBullet de FASE 4 con escala provisional de 43 cm.

## Requisitos Ubuntu

```bash
sudo apt install cmake g++ pkg-config libjson-c-dev python3-venv
```

Se puede reutilizar el entorno virtual anterior:

```bash
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate
```

Si no existe, desde `~/IA_Enjambre_Fase4/sim`:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Terminal 1 — simulador

```bash
cd ~/IA_Enjambre_Fase4/sim
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate  # solo si ese entorno existe
python -u run_demo.py --gui --realtime --serve 7777
```

## Terminal 2 — controlador C++

```bash
cd ~/IA_Enjambre_Fase4
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/swarm_control_demo 127.0.0.1 7777 config/control_demo.json
```

`swarm_gateway_demo` verifica conectividad sin mover brazos. `swarm_control_demo` ejecuta un guion predefinido para `arm_1` y `arm_4`, no una consola interactiva para seis brazos.

## Estado y limitaciones

- El escalado de los brazos de 43 cm es provisional, no un modelo calibrado de los brazos físicos.
- La plataforma giratoria está implementada en la simulación, pero no se garantiza validación física de colisiones ni cinemática.
- No se incluye MPC (previsto para FASE 5).
- La prueba de GUI y de manipulación física debe realizarse en Ubuntu con PyBullet instalado.
- No se incluyen binarios, `build/` ni entornos virtuales para evitar problemas de portabilidad.
- El archivo `config/control_demo.json` contiene las tareas de demostración.

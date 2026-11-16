# FASE 6.1 — Entorno multiagente de aprendizaje (sin entrenamiento)

Se añade `sim/learning/` sobre la versión final de FASE 5.6. Plataforma inmóvil, acciones relativas de 2 cm y simulación DIRECT sin interfaz gráfica. Admite 1, 2 y 6 agentes; observación de 14 valores por agente. El entrenamiento MAPPO queda para FASE 6.2.

## Instalación y prueba en Ubuntu

```bash
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate
cd ~/IA_Enjambre_Fase5/sim
python -m pip install -r requirements.txt
python -m unittest discover -s tests -p 'test_learning_phase61_unit.py' -v
python test_learning_61.py --agents 1 --steps 20
python test_learning_61.py --agents 2 --steps 20
python test_learning_61.py --agents 6 --steps 20
```

No ejecutes simultáneamente otro proceso que intente controlar el mismo mundo. El entorno crea su propia instancia PyBullet en modo DIRECT; no utiliza el servidor TCP del panel.

## Contrato de API

```python
from learning import SwarmLearningEnv
env=SwarmLearningEnv(num_agents=1, seed=42)
obs, info=env.reset(seed=42)
obs, rewards, terminated, truncated, info=env.step({'arm_1':[0,0,1]})
env.close()
```

`reset(seed)` restaura el escenario y genera objetivos reproducibles. `step` avanza 24 pasos de física a 240 Hz. `truncated` limita a 120 decisiones. La plataforma no rota. `info` incluye objetivos, distancias y acciones inválidas.

## Limitaciones conocidas

* Los objetivos y la planificación utilizan preflight geométrico TCP, no IK certificada ni verificación completa de colisiones entre eslabones.
* Los brazos pueden permanecer ocupados mientras se ejecuta una orden; se mantiene su movimiento en las siguientes decisiones.
* No hay política neuronal, entrenamiento, objetos de reciclaje ni rotación.
* Se requiere prueba funcional en Ubuntu con PyBullet instalado; la prueba unitaria de utilidades no equivale a una integración física validada.

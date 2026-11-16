# FASE 6.2 — MAPPO inicial

Esta fase implementa una politica compartida de acciones TCP continuas, un critico
centralizado con estado de dimension fija para 1/2/6 agentes, GAE y PPO. La plataforma
permanece fija. **No ejecuta automaticamente los 1.000 episodios de la fase 6.3.**

## Instalar y probar

```bash
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate
cd ~/IA_Enjambre_Fase5/sim
python -m pip install -r requirements.txt
python -m pytest -q tests/test_mappo_62.py
python train_mappo_62.py --agents 1 --episodes 2
python train_mappo_62.py --agents 2 --episodes 2 --out out/phase62_two
python train_mappo_62.py --agents 6 --episodes 2 --out out/phase62_six
```

Se generan `checkpoint_latest.pt` y `metrics.csv` en la carpeta indicada.

## Alcance y limitaciones

- Este es un entrenador inicial de comprobacion, no un modelo convergido.
- El valor centralizado usa la suma de recompensas del equipo. Los agentes comparten
  politica y ventajas de equipo.
- Los brazos inactivos se rellenan con ceros y una mascara de presencia.
- La validacion de movimiento sigue siendo TCP preliminar, sin IK ni colisiones
  completas. No utilizar en hardware real.
- Los episodios se ejecutan en PyBullet DIRECT, sin ventana.
- La FASE 6.3 introducira el curriculum completo, evaluacion independiente y
  controles de calidad para el entrenamiento de 1.000 episodios.

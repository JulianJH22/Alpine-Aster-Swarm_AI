# Geometría v0.4 — preliminar

- Disco fijo de radio 0,20 m y anillo hexagonal de circunradio 0,60 m.
- Holgura radial 0,005 m, superficies a z=0 m.
- Seis sectores giratorios, independientes del disco central.
- Bases de referencia en seis ángulos separados 60°, a radio 0,36 m (soportes interiores conceptuales).
- `SimWorld.rotate_platform(angle_rad)` posiciona el anillo instantáneamente, sin modelar motor ni barrido.
- El giro se bloquea durante maniobras activas o si un brazo sostiene un objeto.

## Limitaciones críticas

1. Los brazos simulados siguen siendo modelos Franka Panda de PyBullet (alcance configurado 0,78 m), **no** los brazos reales de alcance 0,43 m. No se ha validado su cinemática ni colisiones con soportes.
2. El anillo no transporta objetos mediante física de contacto validada; no colocar piezas sobre él durante giros hasta implementar ese mecanismo.
3. No hay aún cámara móvil, cambiador de herramientas, cintas ni control MARL implementados.
4. Las zonas son marcadores visuales y regiones de observación aproximadas; no representan recipientes físicos.
5. La geometría de sectores se aproxima mediante mallas trianguladas; verificar en PyBullet antes de uso.
6. El movimiento angular es una reubicación cinemática en reposo, no una simulación de movimiento continuo.
7. Las dimensiones son preliminares y deben validarse con URDF y herramientas reales.

## Prueba mínima

```bash
cd sim
pip install -r requirements.txt
python -m pytest tests/test_platform.py -q
python run_demo.py --render out/
```

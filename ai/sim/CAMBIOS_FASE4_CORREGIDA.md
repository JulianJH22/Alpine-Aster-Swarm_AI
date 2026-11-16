# FASE 4 — Correcciones autorizadas

## A. Cierre del demo
- `run_demo.py`: `error_m=None` se imprime como `N/A`; no se cambia el resultado de la maniobra.

## B. Escala geométrica provisional
- `arm.py`: `loadURDF(..., globalScaling=...)` aplica escala a geometría, articulaciones y cinemática del URDF cargado por PyBullet.
- `scenario.json`: factor 0.43/0.78 = 0.55128205; alcance nominal 0.43 m; se ajustan parámetros lineales de TCP, altura de hombro, aproximación y pinza.
- No se altera la masa, carga máxima, velocidad, pares o límites articulares: **no representan el robot real escalado**.
- El URDF Panda es un sustituto provisional; debe reemplazarse por el URDF real cuando exista.

## C. Montaje y accesibilidad
- Se mantiene radio de montaje de hombros de 0.36 m (seis posiciones a 60 grados), evitando cambios no fundamentados en la plataforma.
- Distancia entre hombros vecinos: 0.36 m; alcance nominal desde cada hombro: 0.43 m. La accesibilidad geométrica aproximada NO demuestra alcanzabilidad cinemática ni ausencia de colisiones.
- Se conserva radio de disco fijo de 0.20 m y anillo hasta 0.60 m. Parte del anillo queda fuera del alcance nominal de algunos brazos en ciertas orientaciones.

## D. Validación
- Se agregan pruebas estáticas de escala, configuración y formato; las pruebas físicas existentes requieren PyBullet.
- **No se pudo ejecutar PyBullet en el entorno de preparación**, por lo que no se garantiza el éxito de agarres, ausencia de colisiones o apertura gráfica en Ubuntu.
- La escena tiene objetos de 0.04 m y pinza abierta escalada con ancho total nominal de 0.0441 m, margen muy pequeño.
- Se recomienda probar `python -m pytest -q tests` y `python -u run_demo.py --gui --realtime`.
- El escenario de giro sigue siendo cinemático; no se ha cambiado.

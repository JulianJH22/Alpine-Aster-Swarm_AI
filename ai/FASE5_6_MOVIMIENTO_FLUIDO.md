# FASE 5.6 — Movimiento manual fluido

En la pestaña **Movimiento manual**, activa **Simular movimiento automáticamente y de forma fluida** (activada por defecto). Tras comprobar la trayectoria y confirmar el entorno, pulsa **Enviar movimiento**. El panel enviará la orden y avanzará automáticamente el tiempo de simulación en pasos pequeños (por defecto 50 ms), dejando tiempo a la ventana PyBullet para dibujar el movimiento. El botón **Avanzar física** continúa disponible si se desactiva esta opción.

**Nota:** el bucle de avance ocupa la petición del panel hasta terminar el movimiento; no es control en tiempo real estricto. Se mantiene el planificador preliminar de colisiones y no se habilita entrenamiento MAPPO. No cierres PyBullet durante la ejecución.

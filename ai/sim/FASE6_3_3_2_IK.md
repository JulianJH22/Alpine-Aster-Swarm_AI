# Fase 6.3.3.2 — Control de frecuencia IK (experimental)

Se mantiene la física, las comprobaciones de seguridad y los comandos articulares de PyBullet.
Se reduce opcionalmente la frecuencia de resolución IK **solo en `_move_to`**; el objetivo articular anterior permanece activo en los subpasos intermedios. No se modifican las redes ni las recompensas.

## Uso

Desde `sim`:

```bash
python compare_ik_6332.py --agents 6 --episodes 10
```

También es posible usar `SWARM_IK_STRIDE=2 python train_mappo_63.py ...` (revisar los argumentos propios del entrenador). `SWARM_IK_STRIDE=1` recupera el comportamiento anterior. El valor máximo permitido es 8. No habilitar cadencias superiores en hardware real sin pruebas.

## Advertencias

* Esto es una **reducción de frecuencia IK**, no una caché de soluciones IK (el solucionador puede depender del estado articular).
* Los resultados de `stride=2` o `4` pueden diferir de `1` en precisión, trayectoria, fallos y tasa de éxito.
* No se garantiza que el ahorro en IK reduzca el tiempo total de aprendizaje ni que preserve el control de colisiones.
* Antes de adoptar un stride, comparar éxito individual, error TCP final, límites, contactos y tiempo hasta alcanzar una tasa de éxito equivalente.
* No se ha ejecutado PyBullet en el entorno de construcción; solo verificación sintáctica.

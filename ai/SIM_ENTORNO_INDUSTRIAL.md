# Celula industrial hexagonal - cambios en el simulador

Seis brazos `arm_1..arm_6` en soportes que nacen en los seis vertices de la estructura hexagonal. Los cuerpos roboticos se mantienen en radio 0.36 m, porque el alcance nominal configurado es 0.43 m. El anillo clasificador tiene seis sectores y gira alrededor de la mesa circular fija (radio 0.20 m). Seis cintas VISUALES salen perpendicularmente de los seis lados del hexagono.

Dos railes VISUALES independientes: el exterior (radio 0.69 m, altura 0.78 m) con herramientas alineadas; el interior (radio 0.20 m, altura 1.05 m) con camara, suspendido desde una estructura superior, sin soportes sobre la mesa.

## Limites actuales

- Cintas, railes, herramientas, camara y soportes son **representaciones visuales sin colision**; no transportan ni escanean ni cambian herramientas.
- El giro del anillo y el registro manual de carga siguen funcionando como en el codigo original.
- El brazo sigue siendo Panda escalado; falta comprobar alcance, orientaciones y ausencia de colisiones con PyBullet en ejecucion.
- No se ha modificado el nucleo C++, gateway ni panel.

## Comprobacion

`cd sim && python -m pytest tests/test_industrial_cell.py -q`

Para una imagen: usar `SimWorld('config/scenario.json', gui=False).render('vista.png', eye=(1.8,-1.8,1.5))` en un entorno con pybullet instalado.

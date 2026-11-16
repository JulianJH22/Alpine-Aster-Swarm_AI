# IA_Enjambre_Simulación

## Demostración de desmontaje y clasificación con seis brazos

Proyecto visual autónomo basado en **PyBullet** y en el mismo modelo **Franka Panda escalado** que utiliza la Fase 6 del proyecto. Está pensado para una presentación: muestra un **único objeto en el centro**, dos brazos que lo sujetan, otros dos que retiran componentes, y otros dos que clasifican las piezas. Una plataforma de **seis sectores** gira y descarga las piezas en **dos cintas transportadoras animadas**.

> **Alcance de esta versión:** demostración coreografiada, visual y cinemática. La IA entrenada en Fase 6 no controla todavía el desmontaje; las piezas se acoplan y trasladan programáticamente. No se simulan agarres por contacto, desmontaje mecánico real, apriete de tornillos, cámaras ni seguridad física certificada. La escena no sirve por sí sola para validar ausencia de colisiones.

## 1. Instalación y arranque (WSL / Ubuntu)

Descomprime el ZIP. La carpeta del proyecto se llama exactamente `IA_Enjambre_Simulación` y **no necesita copiarse a `sim` ni reemplazar tus archivos de entrenamiento**.

En tu WSL, con el entorno virtual del proyecto activado:

```bash
cd ~/IA_Enjambre_Simulación
python -m pip install -r requirements.txt  # solo si te falta alguna dependencia
python simulacion.py --cycles 2 --speed 1.65 --hold 8
```

Si tu entorno virtual se activa con la instalación de Fase 6, puedes usar:

```bash
source ~/IA_Enjambre_Fase6/sim/.venv/bin/activate
# Si tu .venv no está ahí, activa el entorno virtual que ya usabas en Fase 6.
```

También puedes iniciar con `bash iniciar.sh`.

### Opciones útiles

```bash
python simulacion.py --cycles 1 --speed 2.5 --hold 15
python simulacion.py --cycles 3 --speed 1.25
python simulacion.py --cycles 1 --capture-moments  # imágenes de presentación
python simulacion.py --cycles 1 --record           # MP4, si WSL/PyBullet lo soporta
python simulacion.py --check                 # comprobación geométrica sin PyBullet
python simulacion.py --headless --cycles 1 --speed 100 --hold 0
```

La última opción prueba la lógica sin abrir ventana. No sustituye a una prueba visual.

### Controles de la ventana

- **P:** pausar / reanudar.
- **+ / -:** subir / bajar la velocidad de reproducción.
- **0:** recuperar el encuadre original.
- **Ratón:** rotar y ampliar la cámara de PyBullet.

## 2. Qué se verá

1. Un **módulo industrial central** sobre una isla fija; dos submódulos extraíbles de colores diferentes.
2. **Brazos 1 y 2**, marcados en cian: aproximación, cierre de pinzas y sujeción.
3. **Brazos 3 y 4**, marcados en ámbar: aproximación, separación de los submódulos y entrega a puntos intermedios.
4. **Brazos 5 y 6**, marcados en verde: recogida de las piezas, transporte y depósito sobre dos bandejas de la plataforma de seis sectores.
5. Los seis brazos se retiran; la plataforma **gira 150°** hasta alinear las dos bandejas con sus cintas correspondientes.
6. Las piezas pasan visualmente a la **cinta de metales (−Y)** y la **cinta de plásticos (+Y)** y llegan a sus salidas.
7. La plataforma vuelve a su orientación de carga y puede comenzar otro ciclo con un objeto nuevo.

No se muestran nombres de estaciones sobre los brazos. Solo aparecen rótulos discretos de etapa, que pueden ocultarse con `--no-status`.

## 3. Resultados

Los registros se generan en `salida/demo/`:

- `eventos.json`: fases y cambios de estado de las piezas.
- `resumen.json`: ciclos completados, piezas evacuadas y advertencias sobre el alcance del prototipo.
- `captura.png`: captura final de la escena, si está disponible.
- `01_clasificacion.png` y `02_giro_y_salida.png`: capturas opcionales con `--capture-moments`.
- `demo.mp4`: grabación opcional con `--record` cuando la versión de PyBullet/WSLg admite vídeo nativo. Si no está disponible, se conservarán los resultados JSON y PNG.

Usa `--out salida/otro_nombre` para guardar una demostración distinta.

## 4. Arquitectura preparada para IA

```text
IA_Enjambre_Simulación/
├── simulacion.py                 Arranque, opciones y registros
├── iniciar.sh                    Inicio directo
├── requirements.txt
├── enjambre/
│   ├── geometry.py               Geometría, sectores y accesibilidad idealizada
│   ├── robots.py                 Robots Panda, IK y animación de articulaciones
│   ├── scene.py                  Objeto, bandejas, transportadores y efectos visuales
│   ├── control.py                Contrato Controller / Observation / RobotCommand
│   └── simulator.py              Orquestador y fases de producción
├── tests/                        Validación geométrica y del contrato
├── INTEGRACION_IA.md             Próximo desarrollo
└── salida/                       Archivos generados
```

El contrato en `enjambre/control.py` diferencia observaciones de escena y comandos. El controlador `ScriptedController` reproduce la coreografía. Un futuro adaptador de IA podrá implementar `Controller.decide(obs)` después de añadir observaciones y acciones de desmontaje reales.

## 5. Precisión geométrica y limitaciones

La configuración anterior tenía bases a **0,68 m** del centro y un alcance nominal de **0,43 m**: no llegaban a la zona central. Esta demostración emplea bases a **0,49 m** y puntos de interacción situados en las caras de la pieza central a **0,15 m** aproximadamente. Todos los objetivos esenciales satisfacen **la condición necesaria de alcance horizontal idealizado** de 43 cm. No es una prueba de IK tridimensional, volumen barrido, seguridad de pares ni colisión entre brazos.

En esta maqueta los manipuladores tienen geometría Panda y se mueven por interpolación de articulaciones. La sujeción, separación y transporte de objetos se hacen con cambios de estado y poses **programados**. Hasta incorporar detección de contactos/constraints y resolver colisiones, estos eventos se entienden como **representación visual**, no como manipulación física demostrada.

## 6. Solución de problemas

- **No aparece ventana:** comprueba WSLg, `echo $DISPLAY` y que PyBullet GUI funcione en WSL.
- **Movimiento demasiado lento:** usa `--speed 2.5` o `--fps 24`; evita ejecutar simultáneamente un entrenamiento pesado.
- **Articulaciones extrañas o solapamientos:** baja a `--speed 1` para inspeccionar, prueba `--cycles 1`, revisa la versión de PyBullet y los registros. Es una limitación del modo cinemático de presentación.
- **`ModuleNotFoundError: pybullet`:** activa el mismo `.venv` usado para la simulación previa, o instala `python -m pip install -r requirements.txt`.


# Interfaz preparada para integración de IA — siguiente fase

## Estado actual

La coreografía completa puede ejecutarse sin depender de los checkpoints. El sistema distingue seis agentes y modela un ciclo de dos piezas, giro y dos cintas. **No hay un modelo BC/MAPPO conectado al desmontaje en esta entrega.**

La política BC previa obtuvo resultados favorables en **posicionamiento TCP local**, no en reconocimiento, agarre, extracción o clasificación física. Conectarla directamente a las nuevas tareas daría una impresión errónea de sus capacidades.

## Contrato implementado

`enjambre/control.py` proporciona:

- `CellObservation`: tiempo, fase, TCP de seis robots, posiciones de piezas, ángulo de plataforma, carga y contadores.
- `RobotCommand`: ID de robot, objetivo TCP, apertura/cierre de pinza y duración.
- `Controller`: interfaz `decide(observation) -> Sequence[RobotCommand]`.
- `ScriptedController`: implementación de demostración, intercambiable.
- `FutureAIController`: marcador explícito de integración, todavía no implementado.

La cinematografía utiliza los mismos IDs `brazo_1` ... `brazo_6`. El programador de alto nivel debe garantizar que la plataforma no gira cuando hay brazos trabajando en ella.

## Ruta de desarrollo

1. **Manipulación física verificable:** pieza dinámica, pinzas/constraints, detección real de agarre y liberación, fuerzas y contactos, extracción de uniones desmontables.
2. **Seguridad e IK:** zonas de exclusión, distancias entre manipuladores, comprobación de trayectorias y singularidades, paradas si un brazo invade la zona de otro.
3. **Control supervisado de tareas:** mapa de objetos, asignación de brazos, clasificación por material, coordinación de plataforma y cintas.
4. **Adaptación de entrenamiento:** nueva función de observación, espacios de acción y recompensas; reutilizar BC como inicio de habilidad de posicionamiento si las dimensiones y cinemática lo permiten. Entrenar maniobras de agarre y desmontaje con currículo y evaluación separada.
5. **Interfaz/HMI:** panel de estado, modo manual/IA, reproducción de episodios, telemetría de colisiones y calidad, selección de checkpoint validado.

No confundir los porcentajes de éxito de posicionamiento anteriores con éxito en desmontaje o clasificación.

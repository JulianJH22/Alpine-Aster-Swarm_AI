# Plataforma v0.5 — movimiento angular progresivo

## Cambios
- `request_platform_rotation(angle_rad)` solicita giro al ángulo absoluto (radianes) por recorrido más corto.
- `step()` avanza la orientación a velocidad configurable `sorting_platform.angular_speed_deg_s` (20°/s por defecto).
- `platform_moving` informa del estado. Se bloquean nuevas solicitudes durante el giro.
- `register_platform_cargo(object_id)` y `unregister_platform_cargo(object_id)` permiten asociar explícitamente piezas al anillo.
- Las piezas registradas se trasladan de manera cinemática manteniendo su radio, altura y orientación global.
- Se mantiene `rotate_platform` como salto instantáneo de compatibilidad, solo en reposo.

## Límites de validación y seguridad
- El transporte es **cinemático, no físico**: no simula fricción, aceleración, deslizamiento ni caídas.
- No hay detección automática de depósitos ni validación del volumen barrido por los brazos.
- Las piezas deben registrarse explícitamente y depositarse en el radio seguro antes de girar.
- La posición de las piezas se reajusta en cada paso; el motor físico no está implementado.
- Los modelos Panda de 78 cm siguen siendo provisionales y no equivalen a los brazos reales de 43 cm.
- Cámara móvil, herramientas y cintas transportadoras continúan pendientes.
- Para pruebas: `cd sim && pip install -r requirements.txt && python -m pytest tests/test_platform.py tests/test_platform_motion.py -q`.

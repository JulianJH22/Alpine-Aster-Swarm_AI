# Entorno industrial v2 — cambios acotados

- Se eliminan los seis postes verticales que rodeaban el raíl exterior de herramientas.
- Los seis robots se colocan en los radios de los seis vértices, a 0,68 m del centro (más allá del radio de 0,60 m de la plataforma).
- Se eliminan los soportes que cruzaban desde los vértices hacia el interior; quedan pedestales exteriores.
- No se cambian las cintas, mesa fija, sectores, raíles, cámara, lógica de giro ni el núcleo C++.

**Advertencia geométrica:** con 43 cm de alcance desde hombro hasta herramienta y las bases a 68 cm, el centro de la mesa queda fuera del alcance nominal. Esta versión prioriza la ubicación exterior solicitada; se necesitará validar y posiblemente rediseñar la cinemática o el tamaño de la plataforma antes de tareas centrales.

No se ha ejecutado una validación física de PyBullet.

# FASE 5.6 — Límites automáticos y gráficas por maniobra

## Uso

1. Arranca PyBullet: `cd ~/IA_Enjambre_Fase5/sim && python -u run_demo.py --gui --serve 7777`.
2. Arranca Streamlit en otra terminal: `python -m streamlit run panel_web_56.py --server.address 127.0.0.1`.
3. En **Movimiento manual**, elige brazo e introduce X/Y/Z en metros. Si el punto está fuera del alcance de 43 cm medido desde el hombro, se proyecta al borde de la región geométrica. Si el preplanificador rechaza el segmento por alcance, suelo o separación TCP, se acorta hasta el último punto validado. Se muestra el destino efectivo. Si el movimiento no tiene un tramo admisible o el brazo está ocupado, se rechaza.
4. Activa **Iniciar registro nuevo al ejecutar movimiento** para que cada movimiento manual empiece su propio intervalo de medición.
5. En **Registro y gráficas**, pulsa **Nueva maniobra / Reiniciar gráficas** para iniciar otro intervalo sin reiniciar el mundo. Tras la secuencia, pulsa **Guardar y generar gráficas de esta maniobra**. Los datos CSV/JSON y PNG quedan en `sim/out/phase56/maniobras/<fecha>_manual/`.
6. Se mantiene el registro completo previo de FASE 5.5.

## Limitaciones

- El recorte es geométrico (esfera de alcance y suelo) y aplica las restricciones del preplanificador TCP. **No garantiza** IK, límites articulares completos ni ausencia de colisiones entre eslabones.
- El recorte de trayectoria se realiza desde el TCP actual hacia el destino pedido; no busca una trayectoria alternativa alrededor de obstáculos.
- El reinicio de gráficas es lógico: no elimina telemetría histórica del servidor ni altera el estado físico.
- El panel usa `sim/config/scenario.json` para geometría; si cambias esa configuración con el servidor en marcha, reinicia ambos procesos para mantenerla sincronizada.
- No se ha incorporado entrenamiento MAPPO.

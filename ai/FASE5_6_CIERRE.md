# FASE 5.6 — Panel web y cierre de la FASE 5

## Funciones
- Panel Streamlit local `sim/panel_web_56.py`.
- Estado y resumen de reciclaje.
- Macros `test1`, `test2`, `test3` (2/4/6 brazos, secuenciales).
- Previsualización y envío de movimiento, avance físico y finalización de tareas.
- Exportación de episodios, gráficas y descargas JSON/CSV.
- Reinicio explícitamente confirmado.

## Instalación y ejecución

Terminal 1:
```bash
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate
cd ~/IA_Enjambre_Fase5/sim
python -m pip install streamlit matplotlib
python -u run_demo.py --gui --serve 7777
```

Terminal 2:
```bash
source ~/IA_Enjambre_Fase3/sim/.venv/bin/activate
cd ~/IA_Enjambre_Fase5/sim
python -m streamlit run panel_web_56.py --server.address 127.0.0.1
```

Abrir `http://localhost:8501` en el mismo ordenador.

## Prueba de aceptación
1. Consultar Estado: deben aparecer seis brazos.
2. Ejecutar `test1`, después `test2` y `test3`; observar movimientos en PyBullet.
3. Guardar episodio, generar gráficas y descargar JSON/CSV.
4. Confirmar reinicio y comprobar que el panel sigue abierto.
5. Reproducir el episodio mediante `python reproducir_55.py` después de cerrar PyBullet.

## Alcance y limitaciones
- Panel local y síncrono: una macro larga ocupa la sesión de Streamlit hasta terminar.
- No hay streaming de video ni control en tiempo real desde el navegador; la GUI permanece en PyBullet.
- `plan_move` es un preflight preliminar; no verifica colisiones completas de eslabones ni IK completa.
- Reciclaje/desmontaje siguen siendo lógicos, no manipulación física real.
- El registro es cinemático; no reproduce contactos y fuerzas.
- No se ejecuta entrenamiento MAPPO: reservado para la FASE 6.

La FASE 5 queda entregada como **candidata a cierre**, sujeta a completar la prueba de aceptación en el Ubuntu del usuario.

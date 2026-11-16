"""FASE 5.6: panel web local, sin entrenamiento ni cambios de protocolo."""
from __future__ import annotations

import json
import time
from pathlib import Path

import streamlit as st

from control_tareas_54 import GatewayClient, execute
from control_limites_56 import shoulder_from_scenario, clamp_workspace, clamp_to_feasible_segment
from maniobras_56 import export_maneuver

BASE = Path(__file__).resolve().parent
OUT = BASE / 'out' / 'phase55' / 'ultimo'
MANEUVERS = BASE / 'out' / 'phase56' / 'maniobras'
SCENARIO = json.loads((BASE / 'config' / 'scenario.json').read_text(encoding='utf-8'))

st.set_page_config(page_title='IA Enjambre | Control', page_icon='🤖', layout='wide')
st.title('IA Enjambre · Panel de control')
st.caption('FASE 5.6 · Seis brazos · Simulador PyBullet · Sin entrenamiento MAPPO')

with st.sidebar:
    st.header('Conexión')
    host = st.text_input('Servidor', '127.0.0.1')
    port = st.number_input('Puerto TCP', min_value=1, max_value=65535, value=7777, step=1)
    st.info('Inicia PyBullet con: python -u run_demo.py --gui --serve 7777')
    st.caption('El panel es local. No expongas el puerto TCP ni Streamlit a redes no confiables.')


def call_command(tokens):
    client = GatewayClient(host, int(port))
    try:
        return execute(client, tokens)
    finally:
        client.close()


def present(result):
    if isinstance(result, (dict, list)):
        st.json(result)
    else:
        st.write(result)


def new_maneuver(client):
    snapshot = client.call('snapshot')
    st.session_state['maneuver_start'] = snapshot['telemetry_samples']
    st.session_state['maneuver_sim_time'] = snapshot['sim_time']
    st.session_state.pop('maneuver_export', None)


def record_maneuver(client, name='manual'):
    if 'maneuver_start' not in st.session_state:
        raise ValueError('Pulsa Nueva maniobra antes de guardar')
    result = export_maneuver(client, st.session_state['maneuver_start'], MANEUVERS, name)
    st.session_state['maneuver_export'] = result
    return result


def perform(tokens):
    try:
        with st.spinner('Ejecutando ' + ' '.join(tokens) + '…'):
            result = call_command(tokens)
        present(result)
        return result
    except Exception as exc:
        st.error(f'{type(exc).__name__}: {exc}')
        return None


tab_status, tab_tests, tab_motion, tab_data, tab_admin = st.tabs([
    'Estado', 'Pruebas automáticas', 'Movimiento manual', 'Registro y gráficas', 'Escenario'
])

with tab_status:
    if st.button('Actualizar estado', type='primary'):
        perform(['estado'])
    st.caption('La actualización es manual; no se envían órdenes de movimiento al consultar el estado.')
    if st.button('Resumen de reciclaje'):
        perform(['resumen'])

with tab_tests:
    st.write('Las macros elevan 6 cm cada herramienta y la devuelven a su posición original, de forma secuencial.')
    c1, c2, c3 = st.columns(3)
    for col, name, description in zip((c1, c2, c3), ('test1', 'test2', 'test3'),
                                      ('2 brazos', '4 brazos', '6 brazos')):
        with col:
            st.subheader(name)
            st.caption(description)
            if st.button('Ejecutar ' + name, key='run_' + name):
                perform([name])
    st.warning('Las comprobaciones previas son preliminares: no garantizan ausencia de colisiones entre eslabones.')

with tab_motion:
    robot = st.selectbox('Brazo', [f'arm_{i}' for i in range(1, 7)])
    xcol, ycol, zcol = st.columns(3)
    with xcol:
        x = st.number_input('X (m)', value=0.193, format='%.3f')
    with ycol:
        y = st.number_input('Y (m)', value=0.000, format='%.3f')
    with zcol:
        z = st.number_input('Z (m)', value=0.250, format='%.3f')
    requested = [float(x), float(y), float(z)]
    shoulder, reach, floor = shoulder_from_scenario(SCENARIO, robot)
    bounded = clamp_workspace(requested, shoulder, reach, floor)
    target = [robot, *(f'{v:.6f}' for v in bounded)]
    st.caption(f'Límite geométrico automático: alcance {reach*100:.0f} cm desde hombro. '
               'La trayectoria sigue sujeta a validación preliminar (sin IK ni colisiones de eslabones).')
    if any(abs(a-b)>1e-5 for a,b in zip(requested,bounded)):
        st.info('Coordenadas saturadas automáticamente: ' + ', '.join(f'{v:.3f} m' for v in bounded))
    smooth = st.checkbox('Simular movimiento automáticamente y de forma fluida', value=True)
    tick = st.slider('Intervalo de simulación (ms)', min_value=20, max_value=200, value=50, step=10, disabled=not smooth)
    max_duration = st.number_input('Límite de tiempo simulado (s)', min_value=1.0, max_value=120.0, value=20.0, step=1.0, disabled=not smooth)
    st.caption('La simulación avanza en pasos pequeños; la velocidad visual depende del rendimiento de PyBullet.')
    if st.button('Comprobar trayectoria'):
        try:
            client = GatewayClient(host, int(port))
            try:
                state = next(a for a in client.call('snapshot')['states'] if a['id'] == robot)
                feasible, preview = clamp_to_feasible_segment(client, robot, state['tcp']['position'], bounded)
                if feasible is None:
                    st.warning('No hay un tramo válido; el movimiento no se ejecutará.')
                elif any(abs(a-b)>1e-5 for a,b in zip(feasible,bounded)):
                    st.info('Destino limitado por el planificador: ' + str([round(v, 4) for v in feasible]))
                present(preview)
            finally:
                client.close()
        except Exception as exc:
            st.error(str(exc))
    st.checkbox('Confirmo que he comprobado la trayectoria y el entorno', key='confirm_move')
    auto_record = st.checkbox('Iniciar registro nuevo al ejecutar movimiento', value=True)
    if st.button('Enviar movimiento', disabled=not st.session_state.get('confirm_move', False)):
        try:
            client = GatewayClient(host, int(port))
            try:
                state = next(a for a in client.call('snapshot')['states'] if a['id'] == robot)
                feasible, preview = clamp_to_feasible_segment(client, robot, state['tcp']['position'], bounded)
                if feasible is None:
                    st.error('No existe un desplazamiento válido en esa dirección.')
                    st.json(preview)
                elif not preview.get('feasible_preflight'):
                    st.error('Movimiento rechazado por el planificador preliminar')
                    st.json(preview)
                else:
                    if any(abs(a-b)>1e-5 for a,b in zip(feasible,requested)):
                        st.info('Destino aplicado automáticamente: ' + ', '.join(f'{v:.4f}' for v in feasible))
                    if auto_record:
                        new_maneuver(client)
                    submission = execute(client, ['mover', robot, *(f'{v:.6f}' for v in feasible)])
                    if not submission.get('submitted'):
                        st.error('El simulador no ha aceptado la orden')
                        st.json(submission)
                    elif not smooth:
                        st.success('Orden enviada. Utiliza Avanzar física para ejecutarla.')
                        present(submission)
                    else:
                        st.info('Ejecutando movimiento con avance automático…')
                        progress = st.progress(0.0)
                        status_area = st.empty()
                        elapsed = 0.0
                        delta = tick / 1000.0
                        # El gateway no tiene reloj autónomo: avanzar física por pasos
                        # y ceder tiempo a la GUI de PyBullet para visualizar cada uno.
                        while elapsed < max_duration:
                            states = client.call('snapshot')['states']
                            current = next((a for a in states if a['id'] == robot), None)
                            if current is None:
                                raise RuntimeError(f'No existe el brazo {robot} en snapshot')
                            if current['status'] == 'idle' and current['available']:
                                st.success(f'{robot}: movimiento finalizado ({elapsed:.2f} s simulados)')
                                break
                            dt = min(delta, max_duration - elapsed)
                            client.call('step', seconds=dt)
                            elapsed += dt
                            progress.progress(min(elapsed / max_duration, 1.0))
                            status_area.caption(f'{robot} · {elapsed:.2f} s simulados · estado: {current["status"]}')
                            time.sleep(delta)
                        else:
                            st.warning('Se alcanzó el límite de simulación; comprueba el estado del brazo antes de enviar otra orden.')
                        present(client.call('snapshot')['states'])
            finally:
                client.close()
        except Exception as exc:
            st.error(f'{type(exc).__name__}: {exc}')
    step = st.number_input('Avanzar simulación (segundos)', min_value=0.01, max_value=120.0, value=0.2)
    if st.button('Avanzar física'):
        perform(['avanzar', str(step)])
    if st.button('Terminar tareas pendientes'):
        perform(['terminar', '30'])

with tab_data:
    st.subheader('Gráficas independientes por maniobra')
    st.caption('Nueva maniobra reinicia el intervalo de gráficas, no la simulación ni los brazos. '
               'Cada registro usa t=0 y se guarda en una carpeta propia.')
    if st.button('Nueva maniobra / Reiniciar gráficas'):
        try:
            client = GatewayClient(host, int(port))
            try:
                new_maneuver(client)
            finally:
                client.close()
            st.success('Registro reiniciado desde el estado actual, sin reiniciar PyBullet.')
        except Exception as exc:
            st.error(str(exc))
    if 'maneuver_start' in st.session_state:
        st.caption(f'Registro activo desde t={st.session_state.get("maneuver_sim_time",0):.2f} s')
    if st.button('Guardar y generar gráficas de esta maniobra'):
        try:
            client = GatewayClient(host, int(port))
            try:
                result = record_maneuver(client)
            finally:
                client.close()
            st.success('Maniobra guardada: ' + result['folder'])
        except Exception as exc:
            st.error(str(exc))
    saved = st.session_state.get('maneuver_export')
    if saved:
        for file in saved['plots']:
            st.image(file)
        folder = Path(saved['folder'])
        for name, mime in [('episode.json','application/json'), ('metrics.csv','text/csv')]:
            st.download_button('Descargar maniobra: ' + name, (folder/name).read_bytes(),
                               file_name=name, mime=mime, key='maneuver_'+name)
    st.divider()
    st.subheader('Registro completo (FASE 5.5)')
    c1, c2 = st.columns(2)
    with c1:
        if st.button('Guardar episodio'):
            perform(['guardar'])
    with c2:
        if st.button('Generar gráficas'):
            perform(['graficas'])
    for filename, label in [('altura_tcp.png', 'Altura de las herramientas'),
                            ('velocidad_articular.png', 'Velocidad articular')]:
        path = OUT / filename
        if path.is_file():
            st.subheader(label)
            st.image(str(path))
    for filename, mime in [('episode.json', 'application/json'), ('metrics.csv', 'text/csv')]:
        path = OUT / filename
        if path.is_file():
            st.download_button('Descargar ' + filename, path.read_bytes(), file_name=filename, mime=mime)
    st.info('Reproducción 3D: detén el servidor PyBullet y ejecuta desde sim/: python reproducir_55.py')

with tab_admin:
    st.warning('Reiniciar borra el estado y la telemetría del servidor. Guarda el episodio antes.')
    confirmed = st.checkbox('Confirmo que quiero reiniciar el escenario')
    if st.button('Reiniciar simulación', disabled=not confirmed):
        try:
            client = GatewayClient(host, int(port))
            try:
                present(client.call('reset'))
            finally:
                client.close()
        except Exception as exc:
            st.error(str(exc))

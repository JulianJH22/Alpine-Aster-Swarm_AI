"""FASE 5.4: consola interactiva para gestionar tareas del simulador.

No ejecuta aprendizaje ni cambia las restricciones de seguridad del gateway.
"""
from __future__ import annotations

import argparse
import json
import time
import shlex
import socket
import sys
import uuid
from swarm_sim.episode_55 import export_episode, plot_episode
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
class GatewayClient:
    """Cliente JSON-lines independiente de PyBullet (protocolo existente)."""
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=10)
        # El reset y run_until_idle pueden tardar mucho mas de 10 s reales.
        self.sock.settimeout(180)
        self.stream = self.sock.makefile('rwb')
        self.counter = 0

    def call(self, op, **args):
        self.counter += 1
        request = {'id': self.counter, 'op': op, 'args': args}
        self.stream.write((json.dumps(request) + '\n').encode())
        self.stream.flush()
        try:
            line = self.stream.readline()
        except (socket.timeout, TimeoutError, OSError) as exc:
            raise RuntimeError(f'Tiempo de espera agotado o conexión interrumpida en {op}: {exc}') from exc
        if not line:
            raise RuntimeError('El simulador cerró la conexión')
        response = json.loads(line)
        if not response['ok']:
            raise RuntimeError(response['error'])
        return response['result']

    def close(self):
        self.stream.close()
        self.sock.close()

HELP = '''Comandos (coordenadas en metros, marco world):
  ayuda                              Mostrar esta ayuda
  estado                             Estado de los seis brazos y objetos
  resumen                            Progreso de reciclaje
  plan ARM X Y Z                     Comprobar movimiento sin ejecutarlo
  mover ARM X Y Z                    Planificar y enviar movimiento seguro
  agarrar ARM OBJETO                 Enviar orden Grasp
  soltar ARM                         Enviar orden Release
  depositar ARM X Y Z                Enviar orden Sort
  resultado ARM ID                   Consultar ejecución
  avanzar SEGUNDOS                   Avanzar la física
  terminar [SEGUNDOS]                Avanzar hasta que termine (máx. 120 s)
  inspeccionar OBJETO                Inspección lógica
  probar OBJETO                      Prueba funcional lógica
  desmontar OBJETO HERRAMIENTA       clip_tool o screwdriver (lógico)
  clasificar OBJETO MATERIAL ESTADO  plastico/madera/electronico; funciona/no_funciona/sin_determinar
  reiniciar                          Reinicia escenario (solicita confirmación)
  test1                              Secuencia de 2 brazos: subir y regresar
  test2                              Secuencia de 4 brazos: subir y regresar
  test3                              Secuencia de 6 brazos: subir y regresar
  guardar                            Guardar episodio y telemetría en out/phase55/ultimo
  graficas                           Generar gráficas PNG del último episodio guardado
  reproducir                         Mostrar comando para reproducción 3D de la última grabación
  salir                              Cerrar consola, sin cerrar PyBullet
Nota: las operaciones de reciclaje siguen siendo lógicas, no mueven brazos.
'''



def _states(client):
    snapshot = client.call('snapshot')
    return {state['id']: state for state in snapshot['states']}


def _wait_for_idle(client, robot_id, max_sim_seconds=20.0):
    # Avance gradual: permite ver el movimiento en GUI y evita una peticion larga.
    elapsed = 0.0
    while elapsed < max_sim_seconds:
        state = _states(client)[robot_id]
        if state['status'] == 'idle' and state['available']:
            return
        client.call('step', seconds=0.15)
        elapsed += 0.15
        time.sleep(0.035)
    raise RuntimeError(f'{robot_id}: no terminó tras {max_sim_seconds} s simulados')


def _macro(client, count):
    # No reiniciar implicitamente: el usuario controla el estado del mundo.
    ids = [f'arm_{i}' for i in range(1, count + 1)]
    summary = []
    for robot_id in ids:
        state = _states(client)[robot_id]
        if not state['available']:
            summary.append(f'{robot_id}: ocupado; prueba detenida')
            break
        origin = list(state['tcp']['position'])
        lifted = [origin[0], origin[1], origin[2] + 0.06]
        for label, target in [('subir', lifted), ('regresar', origin)]:
            preview = client.call('plan_move', robot_id=robot_id, target=target)
            if not preview.get('feasible_preflight'):
                summary.append(f"{robot_id} {label}: rechazado ({preview.get('reason', 'sin motivo')})")
                return '\n'.join(summary)
            execution_id = uuid.uuid4().hex
            command = {'execution_id': execution_id, 'task_id': 'demo_macro_54',
                       'robot_id': robot_id, 'type': 'Move',
                       'target': {'position': target, 'orientation': None, 'frame': 'world'}}
            submitted = client.call('submit_planned', command=command)
            if not submitted.get('submitted'):
                summary.append(f'{robot_id} {label}: orden rechazada')
                return '\n'.join(summary)
            _wait_for_idle(client, robot_id)
            result = client.call('status', robot_id=robot_id, execution_id=execution_id)
            status = result.get('status', 'desconocido')
            summary.append(f'{robot_id} {label}: {status}')
            if status not in ('completed', 'succeeded', 'success'):
                return '\n'.join(summary)
    return '\n'.join(summary)


def execute(client, tokens: list[str], confirm=lambda _: False):
    if not tokens:
        return None
    op, *args = tokens
    if op == 'guardar' and not args:
        # Extraer muestras por páginas: el servidor limita a 1000 por petición.
        samples, index = [], 0
        while True:
            page = client.call('telemetry_samples', since_index=index, limit=1000)
            samples.extend(page['samples'])
            index = page['next_index']
            if index >= page['total']:
                break
        events = client.call('telemetry', since_seq=0)
        target = Path(__file__).resolve().parent / 'out' / 'phase55' / 'ultimo'
        return {'guardado_en': str(target), **export_episode(samples, events, target)}
    if op == 'graficas' and not args:
        target = Path(__file__).resolve().parent / 'out' / 'phase55' / 'ultimo'
        return {'graficas': plot_episode(target / 'episode.json', target)}
    if op == 'reproducir' and not args:
        return 'Cierra la consola y PyBullet antes de abrir una terminal nueva: cd ~/IA_Enjambre_Fase5/sim && python reproducir_55.py'
    if op in ('test1', 'test2', 'test3') and not args:
        return _macro(client, {'test1': 2, 'test2': 4, 'test3': 6}[op])
    if op in ('ayuda', 'help', '?'):
        return HELP
    if op in ('salir', 'exit', 'quit'):
        return 'EXIT'
    if op == 'estado' and not args:
        return client.call('snapshot')
    if op == 'resumen' and not args:
        return client.call('recycling_summary')
    if op in ('plan', 'mover', 'depositar') and len(args) == 4:
        robot, *xyz = args
        target = [float(v) for v in xyz]
        if op == 'plan':
            return client.call('plan_move', robot_id=robot, target=target)
        command = {'execution_id': uuid.uuid4().hex, 'task_id': 'manual_54',
                   'robot_id': robot, 'type': 'Move' if op == 'mover' else 'Sort',
                   'target': {'position': target, 'orientation': None, 'frame': 'world'}}
        if op == 'mover':
            return client.call('submit_planned', command=command)
        return client.call('submit', command=command)
    if op in ('agarrar', 'soltar') and len(args) == (2 if op == 'agarrar' else 1):
        command = {'execution_id': uuid.uuid4().hex, 'task_id': 'manual_54',
                   'robot_id': args[0], 'type': 'Grasp' if op == 'agarrar' else 'Release'}
        if op == 'agarrar':
            command['object_id'] = args[1]
        return client.call('submit', command=command)
    if op == 'resultado' and len(args) == 2:
        return client.call('status', robot_id=args[0], execution_id=args[1])
    if op == 'avanzar' and len(args) == 1:
        seconds = float(args[0])
        if not 0 < seconds <= 120:
            raise ValueError('SEGUNDOS debe estar entre 0 y 120')
        return client.call('step', seconds=seconds)
    if op == 'terminar' and len(args) <= 1:
        seconds = float(args[0]) if args else 120.0
        if not 0 < seconds <= 120:
            raise ValueError('SEGUNDOS debe estar entre 0 y 120')
        return client.call('run_until_idle', timeout_s=seconds)
    if op == 'inspeccionar' and len(args) == 1:
        return client.call('recycling_inspect', component_id=args[0])
    if op == 'probar' and len(args) == 1:
        return client.call('recycling_test', component_id=args[0])
    if op == 'desmontar' and len(args) == 2:
        return client.call('recycling_disassemble', component_id=args[0], tool=args[1])
    if op == 'clasificar' and len(args) == 3:
        return client.call('recycling_classify', component_id=args[0], material=args[1], functional=args[2])
    if op == 'reiniciar' and not args:
        if not confirm('¿Reiniciar el escenario y perder el estado actual? [s/N] '):
            return 'Reinicio cancelado'
        return client.call('reset')
    raise ValueError('Comando o argumentos inválidos. Escribe ayuda.')


def main():
    ap = argparse.ArgumentParser(description='Consola de tareas interactiva FASE 5.4')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=7777)
    args = ap.parse_args()
    try:
        client = GatewayClient(args.host, args.port)
    except OSError as e:
        print(f'No se puede conectar al simulador: {e}\nInicia run_demo.py --gui --serve 7777')
        return 1
    print('Control de tareas FASE 5.4. Escribe ayuda para ver los comandos.')
    try:
        while True:
            try:
                line = input('enjambre> ')
                result = execute(client, shlex.split(line),
                                 confirm=lambda prompt: input(prompt).strip().lower() in ('s', 'si', 'sí'))
                if result == 'EXIT':
                    break
                if result is not None:
                    print(result if isinstance(result, str) else json.dumps(result, indent=2, ensure_ascii=False))
            except (ValueError, RuntimeError, KeyError, OSError) as e:
                print(f'Error: {e}')
            except (EOFError, KeyboardInterrupt):
                print()
                break
    finally:
        client.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

"""FASE 5.5: exportación de telemetría y métricas sin aprendizaje."""
from __future__ import annotations
import csv
import json
from pathlib import Path


def export_episode(samples, events, output_dir):
    folder = Path(output_dir)
    folder.mkdir(parents=True, exist_ok=True)
    if not samples:
        raise ValueError('No hay muestras: ejecuta un movimiento o avanzar antes de guardar')
    with (folder / 'episode.json').open('w', encoding='utf-8') as f:
        json.dump({'format': 'swarm-episode/5.5', 'samples': samples, 'events': events}, f, indent=2, ensure_ascii=False)
    with (folder / 'metrics.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['sim_time_s', 'robot_id', 'tcp_x_m', 'tcp_y_m', 'tcp_z_m', 'speed_rad_s', 'status', 'held_object'])
        for sample in samples:
            for robot_id, arm in sample.get('arms', {}).items():
                pos = arm.get('tcp', [None] * 3)
                vel = arm.get('dq', [])
                writer.writerow([sample.get('sim_time'), robot_id, *pos,
                                 max((abs(float(v)) for v in vel), default=0),
                                 arm.get('status'), arm.get('held')])
    summary = {'samples': len(samples), 'events': len(events),
               'start_s': samples[0].get('sim_time'), 'end_s': samples[-1].get('sim_time'),
               'robots': sorted(samples[-1].get('arms', {}))}
    (folder / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    return summary


def load_episode(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('format') != 'swarm-episode/5.5' or not isinstance(data.get('samples'), list):
        raise ValueError('Archivo de episodio incompatible')
    return data


def plot_episode(path, output_dir):
    """Gráficas de telemetría; no confundir velocidad con error MPC."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    data = load_episode(path)
    samples = data['samples']
    if not samples:
        raise ValueError('Episodio sin muestras')
    folder = Path(output_dir)
    folder.mkdir(parents=True, exist_ok=True)
    ids = sorted({robot for s in samples for robot in s.get('arms', {})})
    for quantity, filename, ylabel in [('z', 'altura_tcp.png', 'Altura TCP (m)'),
                                       ('speed', 'velocidad_articular.png', 'Velocidad articular máxima (rad/s)')]:
        fig, ax = plt.subplots(figsize=(10, 5))
        for robot in ids:
            pairs = [(s['sim_time'], s['arms'][robot]) for s in samples if robot in s.get('arms', {})]
            if not pairs:
                continue
            xs = [t for t, _ in pairs]
            ys = [arm['tcp'][2] if quantity == 'z' else max((abs(float(v)) for v in arm.get('dq', [])), default=0) for _, arm in pairs]
            ax.plot(xs, ys, label=robot)
        ax.set(xlabel='Tiempo de simulación (s)', ylabel=ylabel)
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(folder / filename, dpi=130)
        plt.close(fig)
    return [str(folder / 'altura_tcp.png'), str(folder / 'velocidad_articular.png')]

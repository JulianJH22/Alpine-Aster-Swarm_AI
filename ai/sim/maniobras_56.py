"""Registros independientes de maniobra sin borrar la telemetría del servidor."""
from datetime import datetime
from pathlib import Path
from swarm_sim.episode_55 import export_episode, plot_episode


def export_maneuver(client, since_index, output_root, name='maniobra'):
    samples = []
    index = since_index
    while True:
        page = client.call('telemetry_samples', since_index=index, limit=1000)
        samples.extend(page['samples'])
        index = page['next_index']
        if index >= page['total']:
            break
    if not samples:
        raise ValueError('No hay muestras nuevas para la maniobra')
    start_time = samples[0]['sim_time']
    normalized = [{**s, 'sim_time': s['sim_time'] - start_time} for s in samples]
    events = [e for e in client.call('telemetry', since_seq=0) if e.get('sim_time', e.get('time', -1)) >= start_time]
    folder = Path(output_root) / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + name)
    summary = export_episode(normalized, events, folder)
    plots = plot_episode(folder / 'episode.json', folder)
    return {'folder': str(folder), 'summary': summary, 'plots': plots}

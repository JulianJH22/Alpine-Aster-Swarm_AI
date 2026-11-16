from swarm_sim.episode_55 import export_episode, load_episode


def test_export_and_read(tmp_path):
    samples = [{'sim_time': 0.1, 'arms': {'arm_1': {'tcp': [0.1, 0.2, 0.3], 'dq': [0.1], 'status': 'idle', 'held': None}}, 'objects': {}}]
    summary = export_episode(samples, [{'seq': 0, 'type': 'demo'}], tmp_path)
    assert summary['samples'] == 1
    assert load_episode(tmp_path / 'episode.json')['samples'] == samples
    assert 'arm_1' in (tmp_path / 'metrics.csv').read_text()


def test_empty_rejected(tmp_path):
    import pytest
    with pytest.raises(ValueError):
        export_episode([], [], tmp_path)

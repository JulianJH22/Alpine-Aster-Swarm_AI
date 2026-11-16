def agents_for_episode(episode_index):
    if not 0 <= episode_index < 1000:
        raise ValueError("Indice fuera de los primeros 1000 episodios")
    return 1 if episode_index < 300 else 2 if episode_index < 600 else 6

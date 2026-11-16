import numpy as np

def observation(arm, goal, previous_tcp, elapsed_s):
    tcp = np.asarray(arm.tcp()[0], dtype=np.float64)
    goal = np.asarray(goal, dtype=np.float64)
    error = goal - tcp
    velocity = (tcp - previous_tcp) / max(elapsed_s, 1e-9)
    return np.concatenate([tcp - arm.base_pos, goal - arm.base_pos, error,
                           velocity, [float(arm.active is None), float(np.linalg.norm(error))]]).astype(np.float32)

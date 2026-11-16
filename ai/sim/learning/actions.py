import numpy as np

def normalize_action(action):
    values = np.asarray(action, dtype=np.float64)
    if values.shape != (3,) or not np.isfinite(values).all():
        raise ValueError("La accion debe contener tres numeros finitos")
    return np.clip(values, -1.0, 1.0)

def clamp_reach(start, requested, shoulder, reach, min_z):
    """Satura en esfera de alcance y suelo; NO certifica IK ni colisiones de eslabones."""
    pos = np.asarray(requested, dtype=float).copy()
    pos[2] = max(pos[2], min_z)
    delta = pos - shoulder
    norm = float(np.linalg.norm(delta))
    if norm > reach:
        pos = shoulder + delta * (reach / norm)
    return pos

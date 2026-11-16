"""Saturación geométrica conservadora del TCP; no sustituye IK ni colisiones."""
import math


def shoulder_from_scenario(cfg, robot_id):
    stations = {s['robot_id']: s for s in cfg['stations']}
    station = stations[robot_id]
    hexa = cfg['hexagon']
    angle = math.radians(hexa['rotation_deg']) + station['vertex'] * 2 * math.pi / cfg['polygon_sides']
    radius = hexa.get('shoulder_mount_radius_m', hexa['circumradius_m'])
    model = cfg['robot_models'][station['model']]
    return ([radius * math.cos(angle), radius * math.sin(angle),
             hexa['base_height_m'] + model['shoulder_height_m']], model['reach_m'], cfg['safety']['min_tcp_z_m'])


def clamp_workspace(target, shoulder, reach, min_z):
    """Proyecta al hemisferio de alcance (esfera y suelo), no certifica IK."""
    if not all(math.isfinite(v) for v in [*target, *shoulder, reach, min_z]) or reach <= 0:
        raise ValueError('Parámetros geométricos inválidos')
    if min_z > shoulder[2] + reach:
        raise ValueError('No existe espacio de trabajo sobre el suelo')
    point = [float(target[0]), float(target[1]), max(float(target[2]), min_z)]
    vec = [point[i] - shoulder[i] for i in range(3)]
    distance = math.dist(point, shoulder)
    if distance > reach:
        point = [shoulder[i] + vec[i] * reach / distance for i in range(3)]
        if point[2] < min_z:
            point[2] = min_z
            horizontal = math.hypot(point[0] - shoulder[0], point[1] - shoulder[1])
            allowed = math.sqrt(max(0.0, reach**2 - (min_z - shoulder[2])**2))
            if horizontal > allowed and horizontal > 0:
                scale = allowed / horizontal
                point[0] = shoulder[0] + (point[0] - shoulder[0]) * scale
                point[1] = shoulder[1] + (point[1] - shoulder[1]) * scale
    return point


def clamp_to_feasible_segment(client, robot_id, start, target, iterations=22):
    """Recorta el movimiento desde TCP actual al último punto aprobado por preflight.

    Si el planificador detecta brazo ocupado, plataforma en movimiento o riesgo
    en el punto inicial, no se intenta corregir ni ejecutar.
    """
    preview = client.call('plan_move', robot_id=robot_id, target=target)
    if preview.get('feasible_preflight'):
        return target, preview
    if preview.get('reason') not in ('out_of_reach', 'below_floor', 'tcp_separation_risk'):
        return None, preview
    low, high = 0.0, 1.0
    best = None
    for _ in range(iterations):
        mid = (low + high) / 2
        candidate = [start[i] + mid * (target[i] - start[i]) for i in range(3)]
        result = client.call('plan_move', robot_id=robot_id, target=candidate)
        if result.get('feasible_preflight'):
            low, best = mid, result
        else:
            high = mid
    if best is None or low < 1e-5:
        return None, preview
    return [start[i] + low * (target[i] - start[i]) for i in range(3)], best

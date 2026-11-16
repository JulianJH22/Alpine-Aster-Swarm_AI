"""FASE 5.3: preplanificador predictivo conservador para movimientos TCP.

No es un MPC optimo ni un verificador de colisiones de enlaces. No altera la
simulacion; la ejecucion permanece a cargo de SimRobotArm y su supervisor.
"""
from __future__ import annotations

import math


def plan_tcp(start, target, shoulder, reach_m, other_tcps, min_separation_m,
             min_z_m, max_speed_mps, dt, horizon_steps=12):
    """Predice waypoints de un horizonte recedente y comprueba restricciones TCP.

    Rechaza movimientos que no se puedan certificar con este modelo simple.
    Las distancias a otros TCP se comprueban a lo largo de TODA la recta.
    """
    if not isinstance(horizon_steps, int) or not 1 <= horizon_steps <= 100:
        raise ValueError('horizon_steps debe estar entre 1 y 100')
    vectors = [start, target, shoulder, *other_tcps]
    if any(len(v) != 3 or not all(math.isfinite(float(x)) for x in v) for v in vectors):
        raise ValueError('Coordenadas invalidas o no finitas')
    if not all(math.isfinite(float(v)) and float(v) > 0 for v in
               (reach_m, min_separation_m, max_speed_mps, dt)):
        raise ValueError('Parametros de planificacion invalidos')
    a, b = tuple(map(float, start)), tuple(map(float, target))
    distance = math.dist(a, b)
    n = max(1, math.ceil(distance / max(0.005, min_separation_m / 4)))
    reason = None
    for k in range(n + 1):
        t = k / n
        pos = tuple(a[j] + t * (b[j] - a[j]) for j in range(3))
        if pos[2] < min_z_m:
            reason = 'below_floor'
            break
        if math.dist(pos, shoulder) > reach_m + 1e-9:
            reason = 'out_of_reach'
            break
        if any(math.dist(pos, other) < min_separation_m for other in other_tcps):
            reason = 'tcp_separation_risk'
            break
    step_length = max_speed_mps * dt
    waypoints = []
    for i in range(1, horizon_steps + 1):
        t = min(1.0, i * step_length / distance) if distance > 1e-12 else 1.0
        waypoints.append([a[j] + t * (b[j] - a[j]) for j in range(3)])
    return {
        'feasible_preflight': reason is None,
        'reason': reason,
        'start': list(a), 'target': list(b),
        'distance_m': distance,
        'estimated_duration_s': distance / max_speed_mps,
        'horizon_steps': horizon_steps,
        'waypoints': waypoints,
        'limitations': ['solo TCP; sin colisiones de eslabones',
                        'sin IK ni limites articulares completos',
                        'otros TCP considerados estaticos'],
    }

"""Geometría y trayectorias de la célula de demostración (sin PyBullet)."""
import math

DEG = math.pi / 180.0
ARM_ANGLES = (0, 180, 60, 240, 120, 300)
ARM_ROLES = ('sujecion', 'sujecion', 'desmontaje', 'desmontaje', 'clasificacion', 'clasificacion')
ARM_BASE_RADIUS = 0.49
REACH_NOMINAL = 0.43
TURNTABLE_TRAY_RADIUS = 0.327
COMPONENT_RADIUS = 0.145
HANDOFF_RADIUS = 0.478


def polar(r, deg, z):
    a = deg * DEG
    return (r * math.cos(a), r * math.sin(a), z)


def mix(a, b, t):
    return tuple(x + (y-x)*t for x, y in zip(a,b))


def ease(t):
    x=max(0.0,min(1.0,t))
    return x*x*(3-2*x)


def radial_reach(base_deg, target):
    a=polar(ARM_BASE_RADIUS,base_deg,0)
    return math.hypot(a[0]-target[0],a[1]-target[1])


def tray_angle(arm_angle_deg, rotation_deg):
    return (arm_angle_deg+rotation_deg)%360.0


def validate_essential_geometry():
    """Conservadora: mide sólo distancia horizontal, sin validar IK ni colisiones."""
    tests={
        'holder_1':(0,polar(.155,0,.26)),
        'holder_2':(180,polar(.155,180,.26)),
        'dismantler_3':(60,polar(COMPONENT_RADIUS,60,.275)),
        'dismantler_4':(240,polar(COMPONENT_RADIUS,240,.275)),
        'sorter_5_pickup':(120,polar(HANDOFF_RADIUS,90,.237)),
        'sorter_6_pickup':(300,polar(HANDOFF_RADIUS,270,.237)),
        'sorter_5_tray':(120,polar(TURNTABLE_TRAY_RADIUS,120,.17)),
        'sorter_6_tray':(300,polar(TURNTABLE_TRAY_RADIUS,300,.17)),
    }
    result={k: round(radial_reach(ang,pt),3) for k,(ang,pt) in tests.items()}
    for name,d in result.items():
        if d>REACH_NOMINAL:
            raise ValueError(f'Objetivo fuera de alcance horizontal nominal: {name} ({d} m)')
    assert abs(tray_angle(120,150)-270)<1e-9
    assert abs(tray_angle(300,150)-90)<1e-9
    return result

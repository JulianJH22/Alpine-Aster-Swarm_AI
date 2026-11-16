"""Mundo simulado: hexagono parametrizable, objetos, zonas, entorno y monitor de seguridad."""
from __future__ import annotations

import json
import math
import struct
import time as _time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client

from .recycling import RecyclingModel
from .arm import SimRobotArm
from .telemetry import Telemetry
from .types import Observation, Pose, WorldSnapshot


@dataclass
class Zone:
    name: str
    center: tuple
    half_size: tuple
    rgba: tuple

    def contains(self, pos) -> bool:
        return abs(pos[0] - self.center[0]) <= self.half_size[0] and abs(pos[1] - self.center[1]) <= self.half_size[1]


class SimObject:
    def __init__(self, bc, spec: dict):
        self.bc, self.id = bc, spec["id"]
        self.mass = float(spec["mass"])
        shape, rgba = spec["shape"], spec.get("rgba", [0.7, 0.7, 0.7, 1.0])
        if shape == "box":
            half = [s / 2 for s in spec["size"]]
            col = bc.createCollisionShape(p.GEOM_BOX, halfExtents=half)
            vis = bc.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
            self.grip_width = min(spec["size"][0], spec["size"][1])
        elif shape == "cylinder":
            r, h = spec["radius"], spec["height"]
            col = bc.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=h)
            vis = bc.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=rgba)
            self.grip_width = 2 * r
        elif shape == "sphere":
            r = spec["radius"]
            col = bc.createCollisionShape(p.GEOM_SPHERE, radius=r)
            vis = bc.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=rgba)
            self.grip_width = 2 * r
        else:
            raise ValueError(f"forma no soportada: {shape}")
        orn = p.getQuaternionFromEuler(spec.get("euler", [0, 0, 0]))
        self.body = bc.createMultiBody(self.mass, col, vis, spec["position"], orn)
        bc.changeDynamics(self.body, -1, lateralFriction=1.0, restitution=0.0)
        self.held_by = ""

    def position(self) -> np.ndarray:
        return np.array(self.bc.getBasePositionAndOrientation(self.body)[0])

    def pose(self) -> Pose:
        pos, orn = self.bc.getBasePositionAndOrientation(self.body)
        return Pose(tuple(pos), tuple(orn))


class SimEnvironment:
    """Implementa IEnvironment."""

    def __init__(self, world: "SimWorld"):
        self.world = world

    def observe_object(self, obj: SimObject) -> Observation:
        w = self.world
        pos = obj.position()
        zone = next((z.name for z in w.zones.values() if z.contains(pos)), "")
        vel = w.bc.getBaseVelocity(obj.body)[0]
        return Observation(obj.id, w.time, True, obj.pose(), tuple(vel), obj.held_by, zone)

    def observe(self) -> WorldSnapshot:
        w = self.world
        return WorldSnapshot(
            sim_time=w.time,
            objects=[self.observe_object(o) for o in w.objects.values()],
            active_collisions=[list(c) for c in sorted(w._colliding)],
            zones=[{"name": z.name, "center": list(z.center), "half_size": list(z.half_size)} for z in w.zones.values()])


class SimWorld:
    def __init__(self, config, gui: bool = False, realtime: bool = False):
        self.cfg = self._load(config)
        self.gui, self.realtime = gui, realtime
        self.bc = bullet_client.BulletClient(connection_mode=p.GUI if gui else p.DIRECT)
        self._build()

    @staticmethod
    def _load(config) -> dict:
        if isinstance(config, dict):
            return json.loads(json.dumps(config))
        return json.loads(Path(config).read_text())

    # --------------------------------------------------------------- build
    def _build(self) -> None:
        cfg, bc = self.cfg, self.bc
        bc.setAdditionalSearchPath(pybullet_data.getDataPath())
        ph = cfg["physics"]
        self.dt = 1.0 / ph["rate_hz"]
        bc.setGravity(*ph["gravity"])
        bc.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSolverIterations=ph["solver_iterations"],
                                     deterministicOverlappingPairs=1)
        bc.setRealTimeSimulation(0)
        self.safety = cfg["safety"]
        self.time, self.step_index = 0.0, 0
        self.telemetry = Telemetry(self.dt, cfg["telemetry"]["sample_hz"])
        self._colliding: set = set()
        self._too_close: set = set()

        bc.loadURDF("plane.urdf")
        self._build_platform()
        self._build_industrial_cell()
        self.zones = {z["name"]: Zone(z["name"], tuple(z["center"]), tuple(z["half_size"]), tuple(z["rgba"]))
                      for z in cfg.get("zones", [])}
        self._build_zone_markers()
        self.objects = {o["id"]: SimObject(bc, o) for o in cfg.get("objects", [])}
        self.recycling = RecyclingModel(cfg.get("recycling_components", []))

        hexa = cfg["hexagon"]
        n, R = cfg["polygon_sides"], hexa["circumradius_m"]
        self.arms = {}
        for st in cfg["stations"]:
            ang = math.radians(hexa["rotation_deg"]) + st["vertex"] * 2 * math.pi / n
            shoulder_radius = hexa.get("shoulder_mount_radius_m", R)
            base = [shoulder_radius * math.cos(ang), shoulder_radius * math.sin(ang), hexa["base_height_m"]]
            self.arms[st["robot_id"]] = SimRobotArm(self, st["robot_id"], cfg["robot_models"][st["model"]],
                                                    base, ang + math.pi)  # +x del brazo mira al centro
        self._build_vertex_brackets()
        self.env = SimEnvironment(self)
        if self.gui:
            bc.resetDebugVisualizerCamera(2.2, 45, -35, [0, 0, 0.35])
        self.telemetry.event(0.0, "world_built", scenario=cfg.get("scenario_name"), dt=self.dt,
                             robots=list(self.arms), objects=list(self.objects))

    def _build_platform(self) -> None:
        """Disco central fijo y seis sectores anulares hexagonales con giro cinemático.

        La geometría es una maqueta: las piezas sobre el anillo NO se transportan
        automáticamente por fricción. Se requiere un modelo de contacto/transportador.
        """
        bc, cfg = self.bc, self.cfg
        hexa = cfg["hexagon"]
        geom = cfg["sorting_platform"]
        self.platform_angle = math.radians(hexa["rotation_deg"])
        self.platform_moving = False
        self.platform_target_angle = None
        self.platform_speed_rad_s = math.radians(float(geom.get("angular_speed_deg_s", 20.0)))
        if self.platform_speed_rad_s <= 0:
            raise ValueError("angular_speed_deg_s debe ser positiva")
        self.platform_cargo = {}  # object_id -> (radio, angulo relativo, z, orientacion)
        self.platform_sector_bodies = []
        self.platform_markers = []
        self.platform_radius = float(hexa["circumradius_m"])
        self.platform_inner_radius = float(geom["work_radius_m"]) + float(geom["gap_m"])
        self.platform_thickness = float(geom.get("thickness_m", 0.015))
        self.platform_z = float(geom.get("surface_z_m", 0.0))
        central_radius = float(geom["work_radius_m"])
        col = bc.createCollisionShape(p.GEOM_CYLINDER, radius=central_radius, height=self.platform_thickness)
        vis = bc.createVisualShape(p.GEOM_CYLINDER, radius=central_radius, length=self.platform_thickness,
                                   rgbaColor=[0.62, 0.83, 0.68, 1])
        self.work_surface_body = bc.createMultiBody(0, col, vis,
            [0, 0, self.platform_z - self.platform_thickness / 2])
        # Cada sector es un prisma poligonal. El borde interior aproxima el círculo.
        for i in range(6):
            lo, hi = i * math.pi / 3, (i + 1) * math.pi / 3
            angles = [lo + (hi-lo)*j/12 for j in range(13)]
            outer = [(self.platform_radius*math.cos(lo), self.platform_radius*math.sin(lo)),
                     (self.platform_radius*math.cos(hi), self.platform_radius*math.sin(hi))]
            # Interpolar el lado recto del hexágono para conservar su silueta.
            outline = [(outer[0][0]*(1-j/12)+outer[1][0]*j/12,
                        outer[0][1]*(1-j/12)+outer[1][1]*j/12) for j in range(13)]
            outline += [(self.platform_inner_radius*math.cos(a),
                         self.platform_inner_radius*math.sin(a)) for a in reversed(angles)]
            count = len(outline)
            vertices = [[x,y,-self.platform_thickness] for x,y in outline] + [[x,y,0] for x,y in outline]
            indices = []
            for j in range(1,count-1):
                indices.extend([count, count+j, count+j+1])
                indices.extend([0,j+1,j])
            for j in range(count):
                k=(j+1)%count
                indices.extend([j,k,count+k, j,count+k,count+j])
            color = geom.get("sector_colors", [[0.42,0.61,0.83,1]]*6)[i]
            visual = bc.createVisualShape(p.GEOM_MESH, vertices=vertices, indices=indices,
                                          rgbaColor=color)
            # Collision mesh estática; el giro se permite solo en reposo.
            collision = bc.createCollisionShape(p.GEOM_MESH, vertices=vertices, indices=indices,
                                                 flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
            body = bc.createMultiBody(0, collision, visual, [0,0,self.platform_z],
                                      p.getQuaternionFromEuler([0,0,self.platform_angle]))
            self.platform_sector_bodies.append(body)

    # --------------------------- celula industrial (geometria visual)
    def _visual_box(self, center, half, rgba, yaw=0.0):
        """Malla visual sin colision: no altera la dinamica existente."""
        vis = self.bc.createVisualShape(p.GEOM_BOX, halfExtents=list(half), rgbaColor=list(rgba))
        body = self.bc.createMultiBody(0, -1, vis, list(center),
                                        p.getQuaternionFromEuler([0, 0, yaw]))
        self.cell_visual_bodies.append(body)
        return body

    def _visual_cylinder(self, center, radius, length, rgba):
        vis = self.bc.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=length,
                                        rgbaColor=list(rgba))
        body = self.bc.createMultiBody(0, -1, vis, list(center))
        self.cell_visual_bodies.append(body)
        return body

    def _visual_beam(self, start, end, width, rgba):
        """Viga orientada entre dos puntos (incluye soportes verticales)."""
        a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        direction = b-a
        length = float(np.linalg.norm(direction))
        if length < 1e-6:
            return
        vis = self.bc.createVisualShape(p.GEOM_BOX, halfExtents=[width/2, width/2, length/2],
                                        rgbaColor=list(rgba))
        rotation = p.getQuaternionFromEuler([0, 0, 0])
        # Alinear eje Z local con vector del tramo.
        z = direction/length
        x = np.cross([0, 1, 0], z)
        if np.linalg.norm(x) < 1e-6:
            x = np.cross([1, 0, 0], z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        matrix = np.stack([x, y, z], axis=1)
        # Cuaternion a partir de matriz via scipy no es necesario: PyBullet
        # permite obtenerlo mediante el eje y angulo de alineacion.
        axis = np.cross([0, 0, 1], z)
        axis_len = np.linalg.norm(axis)
        if axis_len > 1e-8:
            axis /= axis_len
            angle = math.acos(float(np.clip(z[2], -1, 1)))
            rotation = p.getQuaternionFromAxisAngle(axis.tolist(), angle)
        elif z[2] < 0:
            rotation = p.getQuaternionFromEuler([math.pi, 0, 0])
        body = self.bc.createMultiBody(0, -1, vis, ((a+b)/2).tolist(), rotation)
        self.cell_visual_bodies.append(body)

    def _visual_ring(self, radius, height, rgba, segments=72, width=0.012):
        for i in range(segments):
            a, b = 2*math.pi*i/segments, 2*math.pi*(i+1)/segments
            self._visual_beam((radius*math.cos(a), radius*math.sin(a), height),
                              (radius*math.cos(b), radius*math.sin(b), height), width, rgba)

    def _build_industrial_cell(self):
        """Cintas radiales y dos railes independientes; representacion visual.

        Las cintas NO transportan piezas automaticamente y los utiles NO se acoplan
        a los robots: estas funciones requieren una fase posterior de simulacion.
        """
        cell = self.cfg.get('industrial_cell', {})
        self.cell_visual_bodies = []
        self.conveyor_centers = []
        self.tool_rail_radius = float(cell.get('tool_rail_radius_m', 0.69))
        self.camera_rail_radius = float(cell.get('camera_rail_radius_m', 0.20))
        self.camera_rail_height = float(cell.get('camera_rail_height_m', 1.05))
        self.tool_rail_height = float(cell.get('tool_rail_height_m', 0.78))
        self.camera_rail_angle = 0.0
        self.tool_rail_angle = 0.0
        if not cell.get('enabled', True):
            return
        dark = [0.24, 0.29, 0.34, 1]
        steel = [0.65, 0.71, 0.76, 1]
        belt = [0.14, 0.17, 0.20, 1]
        # Seis cintas: perpendiculares a los seis lados, NO a los vertices.
        for i in range(6):
            angle = math.radians(self.cfg['hexagon']['rotation_deg']) + (i+0.5)*math.pi/3
            nx, ny = math.cos(angle), math.sin(angle)
            tx, ty = -ny, nx
            near = self.platform_radius*math.cos(math.pi/6) + 0.035
            length = float(cell.get('conveyor_length_m', 0.65))
            width = float(cell.get('conveyor_width_m', 0.19))
            dist = near+length/2
            center = (dist*nx, dist*ny, self.platform_z+0.020)
            self.conveyor_centers.append(center)
            self._visual_box(center, (length/2, width/2, 0.018), belt, angle)
            for sign in (-1, 1):
                self._visual_box((center[0]+sign*(width/2+0.012)*tx,
                                  center[1]+sign*(width/2+0.012)*ty, center[2]+0.018),
                                 (length/2, 0.009, 0.028), steel, angle)
            for j in range(10):
                d = near+0.025+j*(length-0.05)/9
                self._visual_beam((d*nx-width*tx*.45, d*ny-width*ty*.45, center[2]+.021),
                                  (d*nx+width*tx*.45, d*ny+width*ty*.45, center[2]+.021),
                                  0.009, steel)
            for d in (near+.12, near+length-.12):
                for side in (-1, 1):
                    self._visual_beam((d*nx+side*width*.45*tx,d*ny+side*width*.45*ty,-.02),
                                      (d*nx+side*width*.45*tx,d*ny+side*width*.45*ty,center[2]),
                                      0.018,dark)
        # Sin postes verticales alrededor de la guia de herramientas.
        # La guia se representa suspendida, sin apoyos perimetrales.
        self._visual_ring(self.tool_rail_radius,self.tool_rail_height,steel)
        self._visual_ring(self.camera_rail_radius,self.camera_rail_height,[.32,.69,.91,1])
        # La guia de camara se suspende de la estructura superior/techo,
        # no se apoya en la mesa circular fija.
        for i in range(4):
            a=i*math.pi/2
            x,y=self.camera_rail_radius*math.cos(a),self.camera_rail_radius*math.sin(a)
            self._visual_beam((x,y,self.camera_rail_height),(x,y,1.28),.012,steel)
        # Carro de herramientas en linea tangente al rail exterior.
        self.tool_carriage_bodies=[]
        self.camera_carriage_bodies=[]
        for j in range(7):
            y=(j-3)*.055
            self.tool_carriage_bodies.append(self._visual_box((self.tool_rail_radius,y,self.tool_rail_height-.055),
                                        (.018,.012,.055),[.78,.69,.35,1]))
        self.camera_carriage_bodies.append(self._visual_cylinder(
            (self.camera_rail_radius,0,self.camera_rail_height-.055),.026,.055,[.18,.40,.78,1]))

    def _build_vertex_brackets(self):
        """Pedestales fijos exteriores, uno por vertice; no invaden el anillo."""
        self.vertex_mounts = []
        h = self.cfg['hexagon']
        radius = h.get('shoulder_mount_radius_m', self.platform_radius)
        for st in self.cfg['stations']:
            a = math.radians(h['rotation_deg']) + st['vertex'] * math.pi / 3
            vertex = (self.platform_radius * math.cos(a),
                      self.platform_radius * math.sin(a), .025)
            mount = (radius * math.cos(a), radius * math.sin(a), .025)
            self.vertex_mounts.append((vertex, mount))
            self._visual_box(mount, (.065, .065, .045), [.28, .32, .36, 1])

    @staticmethod
    def _shortest_rotation(target: float, current: float) -> float:
        return (target - current + math.pi) % (2 * math.pi) - math.pi

    def _check_platform_clear(self) -> None:
        if not self.all_idle():
            raise RuntimeError("No se puede girar: hay maniobras activas")
        if any(o.held_by for o in self.objects.values()):
            raise RuntimeError("No se puede girar: hay piezas sujetas")

    def register_platform_cargo(self, object_id: str) -> None:
        """Vincula explícitamente una pieza depositada al anillo (modelo cinemático).

        No detecta automáticamente depósitos, ni valida interferencias de los brazos.
        """
        if self.platform_moving:
            raise RuntimeError("No registrar piezas mientras gira la plataforma")
        obj = self.objects[object_id]
        if obj.held_by:
            raise RuntimeError("La pieza está sujeta por un brazo")
        pos, orn = self.bc.getBasePositionAndOrientation(obj.body)
        r = math.hypot(pos[0], pos[1])
        if not self.platform_inner_radius + 0.005 <= r <= self.platform_radius * math.cos(math.pi/6) - 0.01:
            raise ValueError("La pieza debe estar dentro de la banda segura del anillo")
        self.platform_cargo[object_id] = (r, math.atan2(pos[1], pos[0]) - self.platform_angle, pos[2], orn)
        self.telemetry.event(self.time, "platform_cargo_registered", object_id=object_id)

    def unregister_platform_cargo(self, object_id: str) -> None:
        self.platform_cargo.pop(object_id, None)

    def request_platform_rotation(self, angle_rad: float) -> None:
        """Solicita un giro progresivo por el recorrido angular más corto."""
        if not math.isfinite(angle_rad):
            raise ValueError("Ángulo no finito")
        if self.platform_moving:
            raise RuntimeError("La plataforma ya está girando")
        self._check_platform_clear()
        self.platform_target_angle = angle_rad % (2 * math.pi)
        self.platform_moving = abs(self._shortest_rotation(self.platform_target_angle, self.platform_angle)) > 1e-10
        self.telemetry.event(self.time, "platform_rotation_requested", angle_rad=self.platform_target_angle)

    def _set_platform_angle(self, angle_rad: float) -> None:
        self.platform_angle = angle_rad % (2 * math.pi)
        orientation = p.getQuaternionFromEuler([0, 0, self.platform_angle])
        for body in self.platform_sector_bodies:
            self.bc.resetBasePositionAndOrientation(body, [0, 0, self.platform_z], orientation)
        radius = self.cfg["sorting_platform"].get("marker_radius_m", 0.39)
        for marker, offset in self.platform_markers:
            theta = self.platform_angle + offset
            self.bc.resetBasePositionAndOrientation(marker,
                [radius * math.cos(theta), radius * math.sin(theta), self.platform_z + 0.002], orientation)
        for i, zone in enumerate(self.zones.values()):
            theta = self.platform_angle + (i + 0.5) * math.pi / 3
            zone.center = (radius * math.cos(theta), radius * math.sin(theta))
        for object_id, (r, relative, z, orn) in self.platform_cargo.items():
            obj = self.objects[object_id]
            theta = self.platform_angle + relative
            # Modelo cinemático: conserva orientación mundial y altura de la pieza.
            self.bc.resetBasePositionAndOrientation(obj.body, [r * math.cos(theta), r * math.sin(theta), z], orn)
            self.bc.resetBaseVelocity(obj.body, [0, 0, 0], [0, 0, 0])

    def _advance_platform(self) -> None:
        if not self.platform_moving:
            return
        self._check_platform_clear()
        delta = self._shortest_rotation(self.platform_target_angle, self.platform_angle)
        increment = self.platform_speed_rad_s * self.dt
        if abs(delta) <= increment + 1e-10:
            self._set_platform_angle(self.platform_target_angle)
            self.platform_moving = False
            self.platform_target_angle = None
            self.telemetry.event(self.time, "platform_rotation_completed", angle_rad=self.platform_angle)
        else:
            self._set_platform_angle(self.platform_angle + math.copysign(increment, delta))

    def rotate_platform(self, angle_rad: float) -> None:
        """Compatibilidad: posicionamiento instantáneo, solo en reposo.

        Para simular movimiento temporal utilizar request_platform_rotation y step.
        """
        if not math.isfinite(angle_rad):
            raise ValueError("Ángulo no finito")
        if self.platform_moving:
            raise RuntimeError("La plataforma ya está girando")
        self._check_platform_clear()
        self._set_platform_angle(angle_rad)
        self.telemetry.event(self.time, "platform_positioned", angle_rad=self.platform_angle)

    def _build_zone_markers(self) -> None:
        for i, z in enumerate(self.zones.values()):
            # Marcadores sin colisión, solidarios al sector correspondiente.
            offset = (i+0.5)*math.pi/3
            r = self.cfg["sorting_platform"].get("marker_radius_m", 0.39)
            z.center = (r*math.cos(offset+self.platform_angle), r*math.sin(offset+self.platform_angle))
            vis = self.bc.createVisualShape(p.GEOM_CYLINDER, radius=0.045, length=0.002,
                                            rgbaColor=list(z.rgba))
            marker = self.bc.createMultiBody(0,-1,vis,[*z.center,self.platform_z+0.002])
            self.platform_markers.append((marker,offset))

    def reset(self) -> None:
        """Reinicia el escenario a su condicion inicial (las referencias a brazos/objetos antiguos quedan obsoletas)."""
        self.bc.resetSimulation()
        self._build()

    def close(self) -> None:
        self.bc.disconnect()

    # ---------------------------------------------------------------- tiempo
    def step(self, n: int = 1) -> None:
        for _ in range(n):
            self._advance_platform()
            for arm in self.arms.values():
                arm.update()
            self.bc.stepSimulation()
            self.step_index += 1
            self.time = self.step_index * self.dt
            self._monitor()
            self.telemetry.maybe_sample(self.step_index, self.time, self._sample)
            if self.realtime:
                _time.sleep(self.dt)

    def run_for(self, seconds: float) -> None:
        self.step(int(round(seconds / self.dt)))

    def run_until(self, predicate: Callable[[], bool], timeout_s: float) -> bool:
        t_end = self.time + timeout_s
        while self.time < t_end:
            if predicate():
                return True
            self.step()
        return predicate()

    def all_idle(self) -> bool:
        return all(a.active is None for a in self.arms.values())

    def run_until_idle(self, timeout_s: float = 120.0) -> bool:
        return self.run_until(self.all_idle, timeout_s)

    # ------------------------------------------------------------- seguridad
    def _monitor(self) -> None:
        sf, arms = self.safety, list(self.arms.values())
        for i in range(len(arms)):
            for j in range(i + 1, len(arms)):
                a, b = arms[i], arms[j]
                key = (a.id, b.id)
                # 1) contacto fisico entre brazos
                contacts = [c for c in self.bc.getContactPoints(a.body, b.body) if c[8] <= sf["contact_distance_m"]]
                if contacts and key not in self._colliding:
                    self._colliding.add(key)
                    self.telemetry.event(self.time, "collision", a.id, None, with_robot=b.id,
                                         distance=min(c[8] for c in contacts))
                    a.abort("collision", f"con {b.id}")
                    b.abort("collision", f"con {a.id}")
                elif not contacts:
                    self._colliding.discard(key)
                # 2) separacion minima entre TCPs (solo actua si alguno se esta moviendo)
                d = float(np.linalg.norm(a.tcp()[0] - b.tcp()[0]))
                if d < sf["min_tcp_separation_m"] and (a.moving or b.moving):
                    if key not in self._too_close:
                        self._too_close.add(key)
                        self.telemetry.event(self.time, "separation_violation", a.id, None, with_robot=b.id, distance=d)
                    for arm in (a, b):
                        if arm.moving:
                            arm.abort("separation_violation", f"{a.id}-{b.id} a {d:.3f} m")
                elif d >= sf["min_tcp_separation_m"]:
                    self._too_close.discard(key)

    # ----------------------------------------------------- inyeccion de fallos
    def inject_object_displacement(self, object_id: str, position) -> None:
        """Mueve un objeto de golpe (p. ej. se ha deslizado): sirve para probar errores de agarre."""
        o = self.objects[object_id]
        self.bc.resetBasePositionAndOrientation(o.body, list(position), [0, 0, 0, 1])
        self.telemetry.event(self.time, "fault_object_displaced", None, None, object_id=object_id, position=list(position))

    def inject_comm_loss(self, robot_id: str, duration_s: float) -> None:
        arm = self.arms[robot_id]
        arm.comm_lost_until = self.time + duration_s
        self.telemetry.event(self.time, "fault_comm_loss", robot_id, None, duration_s=duration_s)

    # -------------------------------------------------------------- telemetria
    def _sample(self) -> dict:
        arms = {}
        for a in self.arms.values():
            s = a.state()
            arms[a.id] = {"q": s.joint_positions, "dq": s.joint_velocities, "tcp": list(s.tcp.position),
                          "gripper": s.gripper_opening_m, "status": s.status, "held": s.held_object}
        return {"arms": arms, "objects": {o.id: list(o.position()) for o in self.objects.values()},
                "platform_angle_rad": self.platform_angle, "platform_moving": self.platform_moving,
                "platform_cargo": list(self.platform_cargo)}

    # ------------------------------------------------------------------ render
    def render(self, path, width=960, height=640, eye=(1.1, -1.1, 0.9), target=(0, 0, 0.12)) -> None:
        """Foto del escenario (renderer por CPU, funciona sin pantalla)."""
        view = self.bc.computeViewMatrix(list(eye), list(target), [0, 0, 1])
        proj = self.bc.computeProjectionMatrixFOV(50, width / height, 0.05, 5.0)
        img = self.bc.getCameraImage(width, height, view, proj, renderer=p.ER_TINY_RENDERER)[2]
        rgb = np.reshape(np.array(img, dtype=np.uint8), (height, width, 4))[:, :, :3]
        _save_png(path, rgb)


def _save_png(path, rgb: np.ndarray) -> None:
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) \
        + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b"")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_bytes(png)

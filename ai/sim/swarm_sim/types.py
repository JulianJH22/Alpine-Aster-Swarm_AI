"""Tipos compartidos del gemelo digital.

Equivalentes en Python de `include/swarm/core.hpp` (FASE 3 v0.2), AMPLIADOS segun la
seccion 5 del documento de traspaso. Convenios (pendientes de aprobacion por el equipo):

* metros, radianes, segundos, SI.
* Orientaciones como cuaternion (x, y, z, w). Frame soportado: "world".
* Ciclo de vida: submit -> accepted/rejected -> running -> completed/failed/cancelled.
* Todo estado/observacion lleva `sim_time` y `valid`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, fields, is_dataclass
from enum import Enum
from typing import Any, Optional, Protocol, runtime_checkable


class ManeuverType(str, Enum):
    INSPECT = "Inspect"
    GRASP = "Grasp"
    MOVE = "Move"
    HOLD = "Hold"
    RELEASE = "Release"
    SORT = "Sort"


class ExecutionStatus(str, Enum):
    ACCEPTED = "accepted"
    REJECTED = "rejected"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


TERMINAL_STATUSES = {
    ExecutionStatus.REJECTED,
    ExecutionStatus.COMPLETED,
    ExecutionStatus.FAILED,
    ExecutionStatus.CANCELLED,
}


# ---------------------------------------------------------------- rotaciones
def euler_to_quat(roll: float, pitch: float, yaw: float) -> tuple:
    """Euler (ZYX, como el `Pose` actual de C++) -> cuaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_to_euler(q: tuple) -> tuple:
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, s) if abs(s) >= 1 else math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (roll, pitch, yaw)


# ---------------------------------------------------------------------- pose
@dataclass
class Pose:
    """Posicion (m) + orientacion (cuaternion). `orientation=None` en un COMANDO
    significa "herramienta hacia abajo, alineada con el frente del brazo"."""

    position: tuple = (0.0, 0.0, 0.0)
    orientation: Optional[tuple] = None
    frame: str = "world"

    def to_dict(self) -> dict:
        return {
            "position": [float(v) for v in self.position],
            "orientation": None if self.orientation is None else [float(v) for v in self.orientation],
            "frame": self.frame,
        }

    @staticmethod
    def from_dict(d: dict) -> "Pose":
        if "position" in d:
            o = d.get("orientation")
            return Pose(tuple(d["position"]), None if o is None else tuple(o), d.get("frame", "world"))
        # Formato heredado de C++ (x,y,z,roll,pitch,yaw). Angulos a 0 => sin orientacion.
        r, p, y = d.get("roll", 0.0), d.get("pitch", 0.0), d.get("yaw", 0.0)
        orn = None if (r == 0.0 and p == 0.0 and y == 0.0) else euler_to_quat(r, p, y)
        return Pose((d.get("x", 0.0), d.get("y", 0.0), d.get("z", 0.0)), orn, d.get("frame", "world"))


# ------------------------------------------------------------------ comandos
@dataclass
class ManeuverCommand:
    execution_id: str  # unico por ejecucion
    task_id: str
    robot_id: str
    type: ManeuverType
    object_id: str = ""
    target: Optional[Pose] = None
    max_speed: float = 0.4  # m/s del TCP
    tolerance: float = 0.01  # m
    hold_time: Optional[float] = None  # s; None = mantener hasta cancelar
    timeout_s: Optional[float] = None  # None = valor por defecto de la config

    @staticmethod
    def from_dict(d: dict) -> "ManeuverCommand":
        t = d.get("target")
        return ManeuverCommand(
            execution_id=d["execution_id"],
            task_id=d.get("task_id", d["execution_id"]),
            robot_id=d["robot_id"],
            type=ManeuverType(d["type"]),
            object_id=d.get("object_id", ""),
            target=None if t is None else Pose.from_dict(t),
            max_speed=d.get("max_speed", 0.4),
            tolerance=d.get("tolerance", 0.01),
            hold_time=d.get("hold_time"),
            timeout_s=d.get("timeout_s"),
        )


@dataclass
class ManeuverResult:
    execution_id: str
    task_id: str
    robot_id: str
    status: ExecutionStatus
    detail: str = ""
    sim_time: float = 0.0
    elapsed_s: float = 0.0
    error_m: Optional[float] = None  # error final del TCP respecto al objetivo
    data: dict = field(default_factory=dict)


# -------------------------------------------------------------------- estado
@dataclass
class RobotCapabilities:
    robot_id: str
    maneuvers: list
    reach_m: float
    payload_kg: float
    max_tcp_speed_mps: float
    gripper_max_opening_m: float


@dataclass
class RobotState:
    id: str
    sim_time: float
    valid: bool
    available: bool
    status: str  # idle | running | comm_lost
    tcp: Pose
    joint_positions: list
    joint_velocities: list
    gripper_opening_m: float
    held_object: str = ""
    active_execution: str = ""
    error: str = ""


@dataclass
class Observation:
    object_id: str
    sim_time: float
    valid: bool
    pose: Pose
    linear_velocity: tuple
    held_by: str = ""
    zone: str = ""


@dataclass
class WorldSnapshot:
    sim_time: float
    objects: list
    active_collisions: list = field(default_factory=list)  # pares de brazos en contacto
    zones: list = field(default_factory=list)


def to_plain(obj: Any) -> Any:
    """Dataclasses/enums/tuplas -> estructuras JSON."""
    if isinstance(obj, Pose):
        return obj.to_dict()
    if is_dataclass(obj) and not isinstance(obj, type):
        return {f.name: to_plain(getattr(obj, f.name)) for f in fields(obj)}
    if isinstance(obj, Enum):
        return obj.value
    if isinstance(obj, dict):
        return {k: to_plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_plain(v) for v in obj]
    if hasattr(obj, "tolist"):
        return obj.tolist()
    return obj


# ----------------------------------------------------------------- contratos
@runtime_checkable
class IRobotArm(Protocol):
    def state(self) -> RobotState: ...
    def capabilities(self) -> RobotCapabilities: ...
    def submit(self, cmd: ManeuverCommand) -> ManeuverResult: ...
    def result(self, execution_id: str) -> ManeuverResult: ...
    def execute(self, cmd: ManeuverCommand) -> ManeuverResult: ...  # bloqueante, como C++ v0.2
    def cancel(self, id: str) -> bool: ...


@runtime_checkable
class IEnvironment(Protocol):
    def observe(self) -> WorldSnapshot: ...

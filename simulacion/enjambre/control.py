"""Interfaz desacoplada: la IA se conectará aquí sin rehacer el renderizado.

Los checkpoints BC/MAPPO anteriores NO se cargan en esta demostración: sus
observaciones y acciones no describen desmontaje, agarres ni cintas.
"""
from dataclasses import dataclass, field
from typing import Protocol, Mapping, Sequence


@dataclass(frozen=True)
class RobotCommand:
    robot_id: str              # brazo_1 ... brazo_6
    target_xyz: tuple[float,float,float] | None = None
    gripper: float | None = None  # 0=abierta, 1=cerrada
    duration_s: float = 1.0


@dataclass(frozen=True)
class CellObservation:
    sim_time_s: float
    phase: str
    arms_tcp: Mapping[str,tuple[float,float,float]]
    pieces: Mapping[str,tuple[float,float,float]]
    turntable_angle_deg: float
    active_cargo: Sequence[str]
    counters: Mapping[str,int] = field(default_factory=dict)


class Controller(Protocol):
    """Contrato para controlador externo (más adelante IA)."""
    def decide(self, obs: CellObservation) -> Sequence[RobotCommand]: ...


class ScriptedController:
    """Control coreografiado reproducible; el orquestador define las fases."""
    def __init__(self):
        self.commands: list[RobotCommand] = []

    def set_commands(self, commands: Sequence[RobotCommand]):
        self.commands = list(commands)

    def decide(self, obs: CellObservation) -> Sequence[RobotCommand]:
        return tuple(self.commands)


class FutureAIController:
    """Punto de extensión explícito. No es un modelo entrenado."""
    def decide(self, obs: CellObservation) -> Sequence[RobotCommand]:
        raise NotImplementedError('Conectar adaptador de BC/MAPPO tras entrenar nuevas tareas')

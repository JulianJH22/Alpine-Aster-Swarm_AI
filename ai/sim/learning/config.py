from dataclasses import dataclass

@dataclass(frozen=True)
class LearningConfig:
    action_scale_m: float = 0.02
    control_steps: int = 24  # 0.1 s at 240 Hz
    episode_steps: int = 120
    goal_radius_m: float = 0.025  # calibracion: objetivos cercanos, ampliables
    success_tolerance_m: float = 0.01
    max_tcp_speed_mps: float = 0.1
    invalid_penalty: float = 2.0
    success_bonus: float = 10.0
    progress_gain: float = 40.0  # señal de progreso mas visible; sin recompensa repetida
    step_penalty: float = 0.01

"""Entorno directo multiagente de FASE 6.1. Sin redes neuronales.

Las acciones se interpretan como desplazamientos TCP normalizados. El planificador
existente solo valida alcance, suelo y separacion TCP; NO garantiza IK/colisiones.
"""
from __future__ import annotations
import math
from pathlib import Path
import numpy as np
from .config import LearningConfig
from .actions import normalize_action, clamp_reach
from .observations import observation
from .rewards import reward

class SwarmLearningEnv:
    def __init__(self, num_agents=1, platform_rotation=False, seed=42,
                 scenario=None, config=None, gui=False):
        if num_agents not in (1, 2, 6):
            raise ValueError("num_agents debe ser 1, 2 o 6")
        if platform_rotation:
            raise ValueError("La plataforma permanece fija durante FASE 6.1")
        from swarm_sim.world import SimWorld
        self.cfg = config or LearningConfig()
        self.rng = np.random.default_rng(seed)
        self.scenario = Path(scenario) if scenario else Path(__file__).resolve().parents[1] / 'config/scenario.json'
        self.world = SimWorld(self.scenario, gui=gui, realtime=False)
        self.agents = list(self.world.arms)[:num_agents]
        self.goals = {}
        self.previous = {}
        self.finished = set()
        self.steps = 0
        self.initial_distances = {}
        self.initial_angle = self.world.platform_angle

    def reset(self, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.world.reset()
        self.initial_angle = self.world.platform_angle
        self.steps = 0
        self.finished.clear()
        self.goals = {}
        self.previous = {}
        self.initial_distances = {}
        for aid in self.agents:
            arm = self.world.arms[aid]
            tcp = arm.tcp()[0]
            # Objetivos pequeños en torno a la postura inicial; se verifica el
            # preflight geométrico antes de admitirlos.
            from swarm_sim.mpc_preflight import plan_tcp
            shoulder = arm.base_pos + np.array([0, 0, arm.cfg['shoulder_height_m']])
            others = [a.tcp()[0] for key, a in self.world.arms.items() if key != aid]
            goal = tcp.copy()
            for _ in range(80):
                candidate = tcp + self.rng.uniform(-self.cfg.goal_radius_m, self.cfg.goal_radius_m, 3)
                plan = plan_tcp(tcp, candidate, shoulder, arm.cfg['reach_m'], others,
                                self.world.safety['min_tcp_separation_m'],
                                self.world.safety['min_tcp_z_m'], self.cfg.max_tcp_speed_mps, self.world.dt)
                if plan['feasible_preflight'] and np.linalg.norm(candidate-tcp) >= 0.015:
                    goal = candidate
                    break
            else:
                raise RuntimeError(f"No se pudo generar objetivo viable para {aid}")
            self.goals[aid] = goal
            self.previous[aid] = tcp.copy()
            self.initial_distances[aid] = float(np.linalg.norm(goal-tcp))
        return self._observations(), self._info()

    def _observations(self):
        elapsed = self.cfg.control_steps * self.world.dt
        return {aid: observation(self.world.arms[aid], self.goals[aid],
                                 self.previous[aid], elapsed) for aid in self.agents}

    def _info(self):
        return {'sim_time': self.world.time, 'step': self.steps,
                'platform_angle_rad': self.world.platform_angle,
                'goals': {k: v.tolist() for k, v in self.goals.items()},
                'limitations': ['preflight TCP, no IK ni colisiones completas']}

    def step(self, actions):
        from swarm_sim.mpc_preflight import plan_tcp
        from swarm_sim.types import ManeuverCommand, ManeuverType, Pose
        if self.steps >= self.cfg.episode_steps or len(self.finished) == len(self.agents):
            raise RuntimeError("Episodio terminado: ejecutar reset")
        if set(actions) != set(self.agents):
            raise ValueError("Se requiere exactamente una accion por agente activo")
        previous_distances = {aid: float(np.linalg.norm(self.world.arms[aid].tcp()[0]-self.goals[aid]))
                              for aid in self.agents}
        before = {aid: self.world.arms[aid].tcp()[0].copy() for aid in self.agents}
        invalid = {aid: False for aid in self.agents}
        for aid in self.agents:
            if aid in self.finished:
                continue
            arm = self.world.arms[aid]
            act = normalize_action(actions[aid])
            if arm.active is not None:
                # El movimiento anterior puede continuar durante otro step.
                continue
            shoulder = arm.base_pos + np.array([0, 0, arm.cfg['shoulder_height_m']])
            requested = before[aid] + act * self.cfg.action_scale_m
            target = clamp_reach(before[aid], requested, shoulder, arm.cfg['reach_m'],
                                 self.world.safety['min_tcp_z_m'])
            others = [a.tcp()[0] for key, a in self.world.arms.items() if key != aid]
            plan = plan_tcp(before[aid], target, shoulder, arm.cfg['reach_m'], others,
                            self.world.safety['min_tcp_separation_m'],
                            self.world.safety['min_tcp_z_m'], self.cfg.max_tcp_speed_mps, self.world.dt)
            if not plan['feasible_preflight']:
                invalid[aid] = True
                continue
            cmd = ManeuverCommand(execution_id=f"learn-{self.steps}-{aid}",
                                  task_id=f"episode-{self.steps}", robot_id=aid,
                                  type=ManeuverType.MOVE, target=Pose(tuple(target)),
                                  max_speed=self.cfg.max_tcp_speed_mps, tolerance=0.01,
                                  timeout_s=5.0)
            result = arm.submit(cmd)
            if result.status.value == 'rejected':
                invalid[aid] = True
        self.world.step(self.cfg.control_steps)
        self.steps += 1
        if not math.isclose(self.world.platform_angle, self.initial_angle, abs_tol=1e-9):
            raise RuntimeError("La plataforma se ha movido durante el entrenamiento")
        rewards, terminated, truncated = {}, {}, {}
        already_finished = set(self.finished)
        newly_finished = set()
        for aid in self.agents:
            current = float(np.linalg.norm(self.world.arms[aid].tcp()[0]-self.goals[aid]))
            success = current <= self.cfg.success_tolerance_m and self.world.arms[aid].active is None
            first_success = success and aid not in self.finished
            was_finished = aid in self.finished
            if first_success:
                self.finished.add(aid)
                newly_finished.add(aid)
            rewards[aid] = (0.0 if was_finished else float(reward(
                previous_distances[aid], current, invalid[aid], first_success, self.cfg)))
            terminated[aid] = aid in self.finished
            truncated[aid] = self.steps >= self.cfg.episode_steps and aid not in self.finished
        obs = self._observations()
        self.previous = before
        info = self._info()
        info['invalid_actions'] = invalid
        info['reward_components'] = {aid: {
            'progress': 0.0 if aid in already_finished else self.cfg.progress_gain * (previous_distances[aid] - info_distance),
            'time': 0.0 if aid in already_finished else -self.cfg.step_penalty,
            'invalid': 0.0 if aid in already_finished or not invalid[aid] else -self.cfg.invalid_penalty,
            'success': self.cfg.success_bonus if aid in newly_finished else 0.0,
        } for aid, info_distance in ((k, float(np.linalg.norm(self.world.arms[k].tcp()[0]-self.goals[k]))) for k in self.agents)}
        info['new_successes'] = list(newly_finished)
        info['initial_distances_m'] = dict(self.initial_distances)
        info['distances_m'] = {aid: float(np.linalg.norm(self.world.arms[aid].tcp()[0]-self.goals[aid])) for aid in self.agents}
        return obs, rewards, terminated, truncated, info

    def close(self):
        self.world.close()

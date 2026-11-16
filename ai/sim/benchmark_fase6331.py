"""Perfilado profundo no invasivo de PyBullet, Fase 6.3.3.1.

No cambia la fisica, el entrenamiento, la recompensa ni las acciones.
Las categorias son INCLUSIVAS y algunas se solapan; no sumarlas indiscriminadamente.
"""
import argparse
import json
import statistics
import time
from collections import defaultdict
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch

import numpy as np
import torch
from learning.environment import SwarmLearningEnv
from learning.mappo import MAPPO, PPOConfig
from train_mappo_63 import episode, update
import swarm_sim.mpc_preflight as preflight


class Timings:
    def __init__(self):
        self.samples = defaultdict(list)

    def wrap(self, key, fn):
        def measured(*args, **kwargs):
            start = time.perf_counter()
            try:
                return fn(*args, **kwargs)
            finally:
                self.samples[key].append(time.perf_counter() - start)
        return measured

    def report(self):
        return {key: {
            "calls": len(vals),
            "total_s": sum(vals),
            "mean_ms": 1000 * statistics.mean(vals),
            "max_ms": 1000 * max(vals),
        } for key, vals in sorted(self.samples.items()) if vals}


def run(agents, episodes, seed, output, torch_threads):
    torch.set_num_threads(torch_threads)
    torch.manual_seed(seed)
    model = MAPPO()
    config = PPOConfig()
    optimizer = torch.optim.Adam(model.parameters(), lr=config.learning_rate)
    t = Timings()
    start_all = time.perf_counter()
    env = SwarmLearningEnv(num_agents=agents, seed=seed, gui=False)
    metrics = []
    try:
        with ExitStack() as stack:
            def instrument(obj, name, key):
                original = getattr(obj, name)
                stack.enter_context(patch.object(obj, name, t.wrap(key, original)))

            instrument(env, "reset", "env.reset")
            instrument(env, "step", "env.step")
            instrument(env.world, "reset", "world.reset_rebuild")
            instrument(env.world, "_build", "world.build")
            instrument(env.world, "step", "world.step_all_substeps")
            instrument(env.world, "_advance_platform", "world.platform_update")
            instrument(env.world, "_monitor", "world.monitor")
            instrument(env.world, "_sample", "world.telemetry_sample")
            instrument(env.world.bc, "stepSimulation", "bullet.stepSimulation")
            instrument(env.world.telemetry, "maybe_sample", "telemetry.maybe_sample")
            instrument(preflight, "plan_tcp", "preflight.plan_tcp")
            instrument(model, "act", "network.actor")
            instrument(model, "value", "network.critic")

            # reset() recrea los brazos: instrumentar la clase, no instancias obsoletas.
            from swarm_sim.arm import SimRobotArm
            for name in ("update", "tcp", "_ik", "_servo", "_drive", "_check_limits", "submit"):
                instrument(SimRobotArm, name, "arm." + name)

            for i in range(episodes):
                start = time.perf_counter()
                records, info = episode(env, model, seed + i, train=True)
                t.samples["episode.rollout"].append(time.perf_counter() - start)
                start = time.perf_counter()
                update(model, optimizer, records, agents, config)
                t.samples["training.ppo_update"].append(time.perf_counter() - start)
                metrics.append(info)
    finally:
        env.close()
    elapsed = time.perf_counter() - start_all
    data = {
        "agents": agents, "episodes": episodes, "seed": seed,
        "torch_threads": torch_threads, "device": "cpu",
        "elapsed_total_s": elapsed, "steps_total": sum(m["steps"] for m in metrics),
        "team_success_rate": float(np.mean([m["team_success"] for m in metrics])),
        "timings": t.report(),
        "interpretation": {
            "inclusive": "Los tiempos incluyen funciones hijas: NO sumar categorias anidadas.",
            "world_step": "world.step_all_substeps incluye arm.update, bullet.stepSimulation, monitor y telemetria.",
            "arm_servo": "arm._servo incluye arm._ik y arm._drive.",
            "reset": "world.reset_rebuild incluye world.build; env.reset incluye world.reset_rebuild.",
            "preflight": "preflight.plan_tcp incluye llamadas tanto en reset como en step.",
            "world_arms": "world.step actualiza todos los brazos del escenario, incluso si se entrena con menos agentes.",
            "limitations": "Instrumentacion Python introduce overhead; comparar tendencias y repetir con varias semillas.",
        },
    }
    target = Path(output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(data, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--agents", type=int, choices=[1, 2, 6], default=6)
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--torch-threads", type=int, default=1)
    parser.add_argument("--output", default="out/benchmark_fase6331.json")
    args = parser.parse_args()
    if args.episodes < 1 or args.torch_threads < 1:
        parser.error("episodes y torch-threads deben ser positivos")
    run(**vars(args))

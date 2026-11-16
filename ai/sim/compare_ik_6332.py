"""Compara IK original y cadencias opcionales, en procesos independientes."""
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--agents", type=int, choices=[1, 2, 6], default=6)
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--strides", type=int, nargs="+", default=[1, 2, 4])
    args = parser.parse_args()
    results = []
    for stride in args.strides:
        if not 1 <= stride <= 8:
            parser.error("Los strides deben estar entre 1 y 8")
        output = Path(f"out/ik_stride_{stride}_agents_{args.agents}.json")
        env = dict(os.environ, SWARM_IK_STRIDE=str(stride))
        cmd = [sys.executable, "benchmark_fase633.py", "--agents", str(args.agents),
               "--episodes", str(args.episodes), "--seed", str(args.seed),
               "--output", str(output)]
        print(f"\n>>> IK stride={stride}", flush=True)
        subprocess.run(cmd, env=env, check=True, stdout=subprocess.DEVNULL)
        data = json.loads(output.read_text())
        results.append({"stride": stride, "elapsed_s": data["elapsed_total_s"],
                        "success_rate": data["team_success_rate"],
                        "steps": data["steps_total"],
                        "step_mean_ms": data["timings"]["physics_and_environment_step"]["mean_ms"]})
        print(results[-1], flush=True)
    baseline = results[0]["elapsed_s"]
    for result in results:
        result["speedup_vs_first"] = round(baseline / result["elapsed_s"], 3)
    print(json.dumps(results, indent=2, ensure_ascii=False))
    Path("out/ik_comparison.json").write_text(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()

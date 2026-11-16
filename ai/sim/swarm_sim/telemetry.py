"""Telemetria y eventos con tiempo de simulacion (trazables y reproducibles)."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Callable, Optional


def _round(o, nd=9):
    if isinstance(o, float):
        return round(o, nd)
    if isinstance(o, dict):
        return {k: _round(v, nd) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_round(v, nd) for v in o]
    return o


class Telemetry:
    def __init__(self, dt: float, sample_hz: float):
        self.events: list = []
        self.samples: list = []
        self.sample_every = max(1, round(1.0 / (dt * sample_hz)))

    def event(self, sim_time: float, type: str, robot_id: Optional[str] = None,
              execution_id: Optional[str] = None, **detail) -> dict:
        e = {"seq": len(self.events), "sim_time": sim_time, "type": type,
             "robot_id": robot_id, "execution_id": execution_id, "detail": detail}
        self.events.append(e)
        return e

    def maybe_sample(self, step_index: int, sim_time: float, snapshot_fn: Callable[[], dict]) -> None:
        if step_index % self.sample_every == 0:
            s = snapshot_fn()
            s["sim_time"] = sim_time
            self.samples.append(s)

    def events_of(self, type: str, robot_id: Optional[str] = None) -> list:
        return [e for e in self.events
                if e["type"] == type and (robot_id is None or e["robot_id"] == robot_id)]

    def digest(self) -> str:
        """Hash estable del registro completo: sirve para comprobar determinismo."""
        blob = json.dumps(_round({"events": self.events, "samples": self.samples}),
                          sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(blob.encode()).hexdigest()

    def save_jsonl(self, path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w") as f:
            for e in self.events:
                f.write(json.dumps({"kind": "event", **_round(e)}) + "\n")
            for s in self.samples:
                f.write(json.dumps({"kind": "sample", **_round(s)}) + "\n")

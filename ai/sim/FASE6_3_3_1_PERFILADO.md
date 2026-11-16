# Fase 6.3.3.1 — Perfilado profundo

Desde `sim/` con las dependencias originales instaladas:

```bash
python benchmark_fase6331.py --agents 6 --episodes 10 --output out/perfil_6b.json
python benchmark_fase6331.py --agents 1 --episodes 10 --output out/perfil_1b.json
```

No se modifican redes, física, recompensas ni planificador. El script instrumenta temporalmente métodos mediante `unittest.mock.patch`; al terminar restaura los métodos.

**Lectura:** `world.step_all_substeps` incluye `arm.update`, `bullet.stepSimulation`, `world.monitor`, plataforma y telemetría. `arm._servo` incluye `arm._ik` y `arm._drive`. Los tiempos inclusivos no se deben sumar. `world.reset_rebuild` incluye `world.build`. `preflight.plan_tcp` mide tanto generación de objetivos como acciones. El perfilado añade sobrecarga: contrastar con `benchmark_fase633.py`.

**Hallazgo estructural:** `world.step()` actualiza todos los brazos del escenario, incluso cuando `SwarmLearningEnv` selecciona solamente 1 o 2 agentes. No se cambia todavía para preservar equivalencia física.

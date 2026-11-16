# Final AI model snapshots — Alpine Aster

These checkpoint files come from the available Phase 6 training output. They are **not** trained for the complete disassembly/sorting process.

| File | Original training output | Purpose |
|---|---|---|
| `bc_actor_best.pt` | `out/noche6335/bc/bc_actor_best.pt` | Behavior cloning actor; baseline for local TCP goals |
| `mappo_best.pt` | `out/fine_tune_6337/checkpoint_best.pt` | MAPPO fine-tuning best checkpoint |
| `mappo_latest.pt` | `out/fine_tune_6337/checkpoint_latest.pt` | MAPPO fine-tuning latest checkpoint |

**Evaluation context:** previously reported 100-seed team success at 10 mm tolerance was 100% for all three evaluated policies, but at 5 mm tolerance it was 1%, 1%, and 0% respectively. Do not interpret this as end-to-end autonomous manipulation performance.

**Security:** PyTorch `.pt` files can contain pickled objects. Load only checkpoints you trust. The code uses training-specific model architectures and may require the original Phase 6 dependencies.

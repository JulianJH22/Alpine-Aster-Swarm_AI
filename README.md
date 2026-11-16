# Alpine Aster | IA Enjambre

### Leukerbad Hackathon — Cooperative Robotics & AI

**Team:** Alpine Aster  
**Project:** IA Enjambre  
**Event:** Leukerbad Hackathon

A six-arm PyBullet industrial-cell demonstration featuring coordinated disassembly and material sorting, alongside research into multi-agent reinforcement learning.

> **Status:** The team has successfully run and recorded the visual demonstration. Disassembly and sorting are **scripted/choreographed**. BC and MAPPO policies have been trained for **local TCP positioning**, but do **not** yet control the end-to-end disassembly workflow. Physical grasping and robot safety have not been validated.

## 🎬 Demo

**[Watch the demo](media/README.md)** — add the actual video link before publishing.

Add a real screenshot to `media/captura.png`. The video is not included because it was not supplied in this conversation.

## What the simulation demonstrates

1. **Two arms** stabilize a central object.
2. **Two arms** perform a scripted component-removal sequence.
3. **Two arms** sort the components onto a six-sector rotating platform.
4. The platform rotates and transfers the components to **metal and plastic conveyor belts**.

## Repository structure

```text
IA-Enjambre/
├── simulacion/         # Six-arm PyBullet demo
├── ia/fase6/           # AI training and evaluation
├── docs/               # Results, roadmap, hackathon overview
├── models/             # Selected BC and MAPPO checkpoints
├── media/              # Demo video and screenshot
├── .github/workflows/  # Automated checks
└── README.md
```

## Quick start — simulation (Ubuntu / WSL with GUI)

```bash
cd simulacion
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python simulacion.py --cycles 2 --speed 1.65 --hold 10
```

Run the built-in checks:

```bash
python simulacion.py --check
python -m unittest discover -s tests -v
```

See [simulation instructions](simulacion/README.md) for more details.

## AI research

Training code and experiments are in [`ia/fase6/sim`](ia/fase6/sim). Three selected trained checkpoints are included in [`models/`](models/), with provenance and limitations described in [`models/README.md`](models/README.md).

BC and MAPPO have been evaluated on local TCP-positioning tasks. See [results](docs/RESULTADOS.md).

## Future AI integration

The demo separates scene control in `simulacion/enjambre/control.py`. Our next goal is to connect learned policies to robot observations, actions, and task coordination, then train and evaluate full manipulation workflows. See [integration notes](simulacion/INTEGRACION_IA.md).

## Limitations

- Component extraction and transfer are represented kinematically; physical contacts and grasp forces have not been validated.
- No automated visual classification or fully autonomous AI-driven disassembly yet.
- The simulation does not demonstrate physical robot safety.

## License and attribution

**License not yet selected.** Review third-party asset licenses (including PyBullet/URDF resources) before publication.

## Project documentation

- [Hackathon overview](docs/HACKATHON.md)
- [Results](docs/RESULTADOS.md)
- [Roadmap](docs/ROADMAP.md)
- [Publication checklist](docs/PUBLICACION.md)

**Made by Alpine Aster for the Leukerbad Hackathon.**

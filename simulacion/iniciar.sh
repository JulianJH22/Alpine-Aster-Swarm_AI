#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
python simulacion.py --cycles 2 --speed 1.65 --hold 8 "$@"

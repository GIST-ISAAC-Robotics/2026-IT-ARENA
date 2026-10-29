#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -B artifacts/validation/2026-10-28/motion_uncertainty_v1/diagnose_rotation_geometry.py

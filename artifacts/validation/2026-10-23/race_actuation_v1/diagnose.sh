#!/usr/bin/env bash
set -eo pipefail
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
export PYTHONDONTWRITEBYTECODE=1
python3 -B artifacts/validation/2026-10-23/race_actuation_v1/diagnose.py

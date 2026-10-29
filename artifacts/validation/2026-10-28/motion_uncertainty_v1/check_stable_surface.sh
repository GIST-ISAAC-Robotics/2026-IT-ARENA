#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/motion_uncertainty_v1
test ! -e "$base/stable_surface_contract_v1.log"
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_stable_surface.py" > "$base/stable_surface_contract_v1.log" 2>&1
cat "$base/stable_surface_contract_v1.log"

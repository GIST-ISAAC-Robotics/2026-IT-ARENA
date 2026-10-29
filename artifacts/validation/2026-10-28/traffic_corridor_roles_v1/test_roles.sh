#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
python3 -B -m pytest -q -p no:cacheprovider artifacts/validation/2026-10-28/traffic_corridor_roles_v1/test_role_temporal.py tests/test_corridor_roles.py tests/test_corridor_role_probe.py tests/test_corridor_observer.py tests/test_corner_corridor_module.py

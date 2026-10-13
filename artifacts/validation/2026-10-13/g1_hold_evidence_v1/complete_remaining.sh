#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_hold_evidence_v1
python3 -B scripts/audit_g1_hold_evidence.py artifacts/validation/2026-10-13/g1_curve_candidate_v1/first_curve_s0 --config config/tests/g1_traffic_extensions_v1.json --output "$base/curve_s0.json"
python3 -B scripts/audit_g1_hold_evidence.py artifacts/validation/2026-10-13/g1_dense_stop_v1/initial_s0 --config config/tests/g1_traffic_extensions_v1.json --output "$base/initial_new_s0.json"

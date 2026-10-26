#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH="scripts:src/arena_autonomy:src/arena_vehicle_interface:tests:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
A=artifacts/validation/2026-10-25/m3_execution_review_v1
D="$A/run56_m3_sensor_stress_preflight"
[ ! -e "$D/exit_code.txt" ] || exit 91
cp scripts/validate_g1_traffic.py "$D/validator_source.py"
cp scripts/g1_lidar_stress.py "$D/stress_source.py"
cp tests/test_m3_lidar_stress_cli.py "$D/test_source.py"
cp config/tests/m3_sensor_stress_r0_sha256.json "$D/source_contract.json"
cp config/tests/lidar_uniform30_delay50_jitter20.json "$D/impairment_profile.json"
set +e
python3 -B -m pytest -q tests/test_m3_lidar_stress_cli.py tests/test_g1_lidar_stress.py tests/test_m3_execution_integration.py tests/test_source_provenance_r0_plus_b3.py > "$D/stdout.txt" 2> "$D/stderr.txt"
code=$?
printf '%s\n' "$code" > "$D/exit_code.txt"
exit "$code"

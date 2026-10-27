#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/marker_visibility_v1
test ! -e "$base/run2_tests.log"
/usr/bin/python3 -B "$base/run2_snapshot.py" > "$base/run2_snapshot.log" 2>&1
/usr/bin/python3 -B -m pytest -q tests/test_partial_observation.py tests/test_lidar_safety.py tests/test_source_provenance_r0_plus_b3.py > "$base/run2_tests.log" 2>&1
tail -3 "$base/run2_tests.log"
/usr/bin/python3 -B "$base/run2_snapshot.py" --check > "$base/run2_pre_drive_source.log" 2>&1
set +e
/usr/bin/python3 -B scripts/validate_basic_autonomy.py --autonomy-mode local_pursuit --actuation-mode virtual_mcu --lidar-acquisition sequential --lidar-compensation both --control-rate-hz 50 --disable-speed-bump --render-backend wsl_nvidia --shutdown-mode service --grid-slot 3 --speed-profile g1_compare_1mps --partial-observation-policy bounded_local_v2 --course-marker-visibility blank_all --laps 1 --max-sim-seconds 180 --wall-timeout-s 1800 --stop-wall-timeout-s 90 --output "$base/run2_lap" > "$base/run2_lap.log" 2>&1
validator_exit=$?
/usr/bin/python3 -B scripts/audit_local_pursuit_road.py "$base/run2_lap" > "$base/run2_road.log" 2>&1
road_exit=$?
/usr/bin/python3 -B "$base/run2_snapshot.py" --check > "$base/run2_source_check.log" 2>&1
source_exit=$?
set -e
printf 'validator=%s\nroad=%s\nsource=%s\n' "$validator_exit" "$road_exit" "$source_exit" > "$base/run2_exit.txt"
if (( validator_exit != 0 || road_exit != 0 || source_exit != 0 )); then exit 1; fi

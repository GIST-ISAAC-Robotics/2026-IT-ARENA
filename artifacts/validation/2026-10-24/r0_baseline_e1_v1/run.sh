#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
base=artifacts/validation/2026-10-24/r0_baseline_e1_v1
test ! -e "$base/build.log"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q tests/test_g1_solo_candidate_validation.py tests/test_g1_traffic_eval.py > "$base/tests.log" 2>&1
python3 -B "$base/snapshot.py" > "$base/snapshot.log" 2>&1
colcon build --symlink-install --packages-select arena_autonomy arena_bringup arena_vehicle_interface > "$base/build.log" 2>&1
source install/local_setup.bash
test ! -e "$base/run"
set +e
python3 -B scripts/validate_basic_autonomy.py --autonomy-mode local_pursuit --actuation-mode virtual_mcu --lidar-acquisition sequential --lidar-compensation both --control-rate-hz 50 --disable-speed-bump --render-backend wsl_nvidia --shutdown-mode service --grid-slot 3 --speed-profile g1_compare_1mps --laps 1 --max-sim-seconds 180 --wall-timeout-s 1200 --stop-wall-timeout-s 60 --output "$base/run" > "$base/runner.log" 2>&1
validator_exit=$?
python3 -B scripts/audit_local_pursuit_road.py "$base/run" > "$base/road.log" 2>&1
road_exit=$?
python3 -B "$base/snapshot.py" --check > "$base/source_check.log" 2>&1
source_exit=$?
set -e
printf 'validator=%s\nroad=%s\nsource=%s\n' "$validator_exit" "$road_exit" "$source_exit" > "$base/exit.txt"
tail -3 "$base/runner.log"
if (( validator_exit != 0 || road_exit != 0 || source_exit != 0 )); then exit 1; fi

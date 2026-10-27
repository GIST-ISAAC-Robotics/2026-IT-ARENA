#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/partial_observation_v1
test ! -e "$base/run8_tests.log"
python3 -B "$base/run8_snapshot.py" > "$base/run8_snapshot.log" 2>&1
python3 -B -m pytest -q tests/test_official_start_ros_fixture.py tests/test_pipeline_fault_validation.py tests/test_source_provenance_r0_plus_b3.py > "$base/run8_tests.log" 2>&1
tail -3 "$base/run8_tests.log"
python3 -B scripts/validate_sensor_input_ros.py --partial-observation-policy bounded_side_v1 --output "$base/run8_ros" > "$base/run8_ros.log" 2>&1
cat "$base/run8_ros.log"
python3 -B "$base/run8_snapshot.py" --check > "$base/run8_pre_drive_source.log" 2>&1
test ! -e "$base/run8_lap"
set +e
python3 -B scripts/validate_basic_autonomy.py --autonomy-mode local_pursuit --actuation-mode virtual_mcu --lidar-acquisition sequential --lidar-compensation both --control-rate-hz 50 --disable-speed-bump --render-backend wsl_nvidia --shutdown-mode service --grid-slot 3 --speed-profile g1_compare_1mps --partial-observation-policy bounded_side_v1 --laps 1 --max-sim-seconds 180 --wall-timeout-s 1800 --stop-wall-timeout-s 90 --output "$base/run8_lap" > "$base/run8_lap.log" 2>&1
validator_exit=$?
python3 -B scripts/audit_local_pursuit_road.py "$base/run8_lap" > "$base/run8_road.log" 2>&1
road_exit=$?
python3 -B "$base/run8_snapshot.py" --check > "$base/run8_source_check.log" 2>&1
source_exit=$?
set -e
printf 'validator=%s\nroad=%s\nsource=%s\n' "$validator_exit" "$road_exit" "$source_exit" > "$base/run8_exit.txt"
tail -4 "$base/run8_lap.log"
cat "$base/run8_exit.txt"
if (( validator_exit != 0 || road_exit != 0 || source_exit != 0 )); then exit 1; fi

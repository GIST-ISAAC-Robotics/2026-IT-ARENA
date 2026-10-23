#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_solo_lap_v2
test ! -e "$base/preflight.log"
python3 -B -m pytest -q tests/test_g1_solo_candidate_validation.py tests/test_local_follow_validation.py tests/test_g1_stop_pose_coverage.py tests/test_g1_dense_stop.py tests/test_corner_follow.py tests/test_corner_evidence.py > "$base/preflight.log" 2>&1
if (( $(date -u +%s) >= 1792035000 )); then echo wrap_up_deadline_reached; exit 0; fi
test ! -e "$base/run"
set +e
python3 -B scripts/validate_basic_autonomy.py --g1-follow --g1-boundary-evidence corner_anchor_candidate --g1-occupancy-timing per_ray_candidate --record-follow-decisions --autonomy-mode local_pursuit --actuation-mode virtual_mcu --lidar-acquisition sequential --lidar-compensation both --control-rate-hz 50 --disable-speed-bump --render-backend wsl_nvidia --shutdown-mode service --grid-slot 3 --speed-profile exploratory --laps 1 --max-sim-seconds 180 --wall-timeout-s 1200 --stop-wall-timeout-s 60 --output "$base/run" > "$base/runner.log" 2>&1
validator_exit=$?
python3 -B scripts/replay_follow_trace.py "$base/run/follow_decisions.jsonl" --output "$base/replay.json" > "$base/replay.log" 2>&1
replay_exit=$?
python3 -B scripts/audit_follow_occupancy.py "$base/run/follow_decisions.jsonl" --mode per_ray_candidate --output "$base/occupancy.json" > "$base/occupancy.log" 2>&1
occupancy_exit=$?
python3 -B scripts/audit_local_pursuit_road.py "$base/run" > "$base/road.log" 2>&1
road_exit=$?
set -e
printf 'validator=%s\nreplay=%s\noccupancy=%s\nroad=%s\n' "$validator_exit" "$replay_exit" "$occupancy_exit" "$road_exit" > "$base/exit.txt"
tail -3 "$base/runner.log"
if (( validator_exit != 0 || replay_exit != 0 || occupancy_exit != 0 || road_exit != 0 )); then exit 1; fi

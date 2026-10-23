#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
base=artifacts/validation/2026-10-16/g1_corridor_solo_v1
test ! -e "$base/build.log"
python3 -B "$base/prepare.py" > "$base/preflight.log" 2>&1
colcon build --symlink-install --packages-select arena_autonomy arena_bringup > "$base/build.log" 2>&1
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
python3 -B -c "from arena_autonomy.local_follow import FollowConfig, LocalFollow; from arena_autonomy.corridor_observer import CorridorObserver; c=LocalFollow(FollowConfig(boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate', corridor_fallback_enabled=True)); assert isinstance(c.corridor_observer, CorridorObserver); print('candidate_import_ok')" > "$base/import.log" 2>&1
if (( $(date -u +%s) >= $(date -u -d '2026-10-15 22:30:00 UTC' +%s) )); then echo wrap_up_deadline_reached; exit 0; fi
test ! -e "$base/run"
set +e
python3 -B scripts/validate_basic_autonomy.py --g1-follow --g1-boundary-evidence corner_distance_candidate --g1-occupancy-timing per_ray_candidate --g1-corridor-fallback --record-follow-decisions --autonomy-mode local_pursuit --actuation-mode virtual_mcu --lidar-acquisition sequential --lidar-compensation both --control-rate-hz 50 --disable-speed-bump --render-backend wsl_nvidia --shutdown-mode service --grid-slot 3 --speed-profile exploratory --laps 1 --max-sim-seconds 180 --wall-timeout-s 1200 --stop-wall-timeout-s 60 --output "$base/run" > "$base/runner.log" 2>&1
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

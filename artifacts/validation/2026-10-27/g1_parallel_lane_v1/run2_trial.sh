#!/usr/bin/env bash
# run2: run1_trial.sh와 같은 후보·CLI로 오른쪽 대응 장면 1회. 슬롯0 설정과 오른쪽 overlay를 함께 쓴다.
# 소스가 run1 스냅샷과 같아(957파일) 회귀·빌드는 재실행하지 않고 소스/설정/설치 일치만 확인한다.
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/g1_parallel_lane_v1
test ! -e "$base/run2_right"
test ! -e "$base/run2_exit.txt"
test ! -e "$base/run2_source_manifest.json"
/usr/bin/python3 -B "$base/run1_snapshot.py" --check > "$base/run2_vs_run1_source.log" 2>&1
/usr/bin/python3 -B "$base/run2_config_check.py" > "$base/run2_config_check.log" 2>&1
/usr/bin/python3 -B "$base/run2_install_check.py" > "$base/run2_install_check.log" 2>&1
/usr/bin/python3 -B "$base/run2_snapshot.py" > "$base/run2_snapshot.log" 2>&1
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2|parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run2_preexisting_processes.txt" || true
test ! -s "$base/run2_preexisting_processes.txt"
/usr/bin/python3 -B "$base/run2_snapshot.py" --check > "$base/run2_pre_drive_source.log" 2>&1
set +e
CMD=(/usr/bin/python3 -B scripts/validate_g1_traffic.py --case slow_lead --seed 0 --mode g1 --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --partial-observation-policy bounded_local_v4 --parallel-hold-trial --config config/tests/e3_m3_right_grid_v1.json --actor-overlay config/tests/e3_onroad_right_encounter_v1.json --output "$base/run2_right")
printf '%q ' "${CMD[@]}" > "$base/run2_command.txt"
echo >> "$base/run2_command.txt"
"${CMD[@]}" > "$base/run2_trial.log" 2>&1
trial_exit=$?
/usr/bin/python3 -B "$base/run2_snapshot.py" --check > "$base/run2_source_check.log" 2>&1
source_exit=$?
sleep 3
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2|parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run2_residual_processes.txt"
residual_exit=$?   # grep 1 = 잔존 없음
/usr/bin/python3 -B artifacts/validation/2026-10-27/partial_observation_v1/audit_wheel_centers.py "$base/run2_right" --output "$base/run2_wheel_centers.json" > "$base/run2_wheel_centers.log" 2>&1
wheel_exit=$?
/usr/bin/python3 -B scripts/replay_follow_trace.py "$base/run2_right/follow_decisions.jsonl" --output "$base/run2_right/follow_trace_replay.json" > "$base/run2_replay.log" 2>&1
replay_exit=$?
/usr/bin/python3 -B scripts/traffic_front_target.py "$base/run2_right" --output "$base/run2_front_target.json" > "$base/run2_front_target.log" 2>&1
front_exit=$?
set -e
printf 'trial=%s\nsource=%s\nresidual_grep=%s\nwheel=%s\nreplay=%s\nfront=%s\n' "$trial_exit" "$source_exit" "$residual_exit" "$wheel_exit" "$replay_exit" "$front_exit" > "$base/run2_exit.txt"
if (( source_exit != 0 || residual_exit == 0 || wheel_exit != 0 || replay_exit != 0 || front_exit != 0 )); then exit 2; fi
exit "$trial_exit"

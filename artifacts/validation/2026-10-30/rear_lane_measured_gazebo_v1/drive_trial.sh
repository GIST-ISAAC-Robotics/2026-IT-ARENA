#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
candidate=${1:?off or on required}
export ROS_DOMAIN_ID=230
case "$candidate" in off|on) ;; *) exit 91;; esac
base=artifacts/validation/2026-10-30/rear_lane_measured_gazebo_v1
run=drive_${candidate}
test ! -e "$base/$run"
test ! -e "$base/${run}_trial.log"
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/${run}_preexisting_processes.txt" || true
if [ -s "$base/${run}_preexisting_processes.txt" ]; then echo 'preexisting simulation; not starting' > "$base/${run}_ABORTED.txt"; exit 92; fi
/usr/bin/python3 -B "$base/run_trial.py" --side left --candidate "$candidate" --output "$base/$run" --prepare-only > "$base/${run}_prepare.log" 2>&1
set +e
/usr/bin/python3 -B "$base/run_trial.py" --side left --candidate "$candidate" --output "$base/$run" > "$base/${run}_trial.log" 2>&1
trial_exit=$?
/usr/bin/python3 -B artifacts/validation/2026-10-27/partial_observation_v1/audit_wheel_centers.py "$base/$run" --output "$base/${run}_wheel_centers.json" > "$base/${run}_wheel_centers.log" 2>&1
wheel_exit=$?
/usr/bin/python3 -B scripts/replay_follow_trace.py "$base/$run/follow_decisions.jsonl" --output "$base/$run/follow_trace_replay.json" > "$base/${run}_replay.log" 2>&1
replay_exit=$?
/usr/bin/python3 -B scripts/traffic_front_target.py "$base/$run" --output "$base/${run}_front_target.json" > "$base/${run}_front_target.log" 2>&1
front_exit=$?
/usr/bin/python3 -B "$base/audit_transition.py" "$base/$run" > "$base/${run}_transition.log" 2>&1
transition_exit=$?
sleep 3
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/${run}_residual_processes.txt" || true
printf 'trial=%s\nwheel=%s\nreplay=%s\nfront=%s\ntransition=%s\n' "$trial_exit" "$wheel_exit" "$replay_exit" "$front_exit" "$transition_exit" > "$base/${run}_exit.txt"
exit "$trial_exit"

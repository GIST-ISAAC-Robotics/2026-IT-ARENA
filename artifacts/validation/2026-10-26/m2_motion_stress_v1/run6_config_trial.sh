#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
BASE=artifacts/validation/2026-10-26/m2_motion_stress_v1
O="$BASE/run6_config_trial"
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
python3 -B "$BASE/run_config_check.py" before "$O" > "$O/preflight_stdout.txt" 2> "$O/preflight_stderr.txt"
pre=$?
printf '%s\n' "$pre" > "$O/preflight_exit_code.txt"
if [ "$pre" -ne 0 ]; then exit "$pre"; fi
python3 -B scripts/validate_g1_traffic.py --case curve_stop_restart --seed 0 --mode m3 --m3-command-fallback --m3-release-to-baseline --m3-small-center-straight --m3-straight-surface-commitment --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --config config/tests/g1_curve_stop_restart_v1.json --motion-test-profile config/tests/motion_combined.json --output "$O/run" > "$O/run_stdout.txt" 2> "$O/run_stderr.txt"
run=$?
printf '%s\n' "$run" > "$O/run_exit_code.txt"
date -Is > "$O/run_end.txt"
python3 -B "$BASE/run_config_check.py" after "$O" > "$O/postflight_stdout.txt" 2> "$O/postflight_stderr.txt"
post=$?
printf '%s\n' "$post" > "$O/postflight_exit_code.txt"
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
if [ -f "$O/run/follow_decisions.jsonl" ]; then
  python3 -B scripts/replay_follow_trace.py "$O/run/follow_decisions.jsonl" --output "$O/follow_trace_replay.json" > "$O/replay_stdout.txt" 2> "$O/replay_stderr.txt"
  printf '%s\n' "$?" > "$O/replay_exit_code.txt"
fi
if [ -f "$O/run/report.json" ]; then
  python3 -B scripts/surface_contact_eval.py "$O/run" --output "$O/surface_contact_eval.json" > "$O/surface_stdout.txt" 2> "$O/surface_stderr.txt"
  printf '%s\n' "$?" > "$O/surface_exit_code.txt"
  PYTHONPATH="artifacts/validation/2026-10-25/m3_execution_review_v1:$PYTHONPATH" python3 -B artifacts/validation/2026-10-25/m3_execution_review_v1/evaluate_curve_stop.py "$O/run" --output "$O/curve_hold_eval.json" > "$O/curve_stdout.txt" 2> "$O/curve_stderr.txt"
  printf '%s\n' "$?" > "$O/curve_exit_code.txt"
fi
date -Is > "$O/end.txt"
if [ "$post" -ne 0 ]; then exit "$post"; fi
exit "$run"

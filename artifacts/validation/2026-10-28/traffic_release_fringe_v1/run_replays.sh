#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/traffic_release_fringe_v1
test ! -e "$base/off_replay.log"
test ! -e "$base/recording_comparison"
set +e
python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course/follow_decisions.jsonl --output "$base/off_replay.json" > "$base/off_replay.log" 2>&1
off=$?
if [ "$off" -eq 0 ]; then
  python3 -B "$base/compare_recording.py" > "$base/recording_comparison.log" 2>&1
  comparison=$?
else
  comparison=99
fi
printf 'off=%s\ncomparison=%s\n' "$off" "$comparison" > "$base/replays.exit"
exit "$((off || comparison))"

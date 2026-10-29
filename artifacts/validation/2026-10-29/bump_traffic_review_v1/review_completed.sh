#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/bump_traffic_review_v1
test ! -e "$base/review_completed.log"
set +e
/usr/bin/python3 -B "$base/probe_mixed_evidence.py" > "$base/review_completed.log" 2>&1
probe=$?
/usr/bin/python3 -B "$base/plot_rolling_result.py" >> "$base/review_completed.log" 2>&1
plot=$?
printf 'probe=%s\nplot=%s\n' "$probe" "$plot" > "$base/review_completed.exit"
if [ "$probe" -ne 0 ]; then exit "$probe"; fi
exit "$plot"

#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-03/solo_straight_closed_loop_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [ -e "$A/run1" ] || [ -e "$A/run1_registered.json" ]; then exit 3; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/preflight_processes.log"; then exit 3; fi
bash "$ENV" /usr/bin/python3 -B "$A/run.py" --prepare-only > "$A/prepare.log" 2>&1
rc=$?; echo "$rc" > "$A/prepare.exit"; if [ "$rc" -ne 0 ]; then exit "$rc"; fi
bash "$ENV" /usr/bin/python3 -B "$A/run.py" > "$A/wrapper.log" 2>&1
rc=$?; echo "$rc" > "$A/runtime.exit"
if [ -f "$A/run1/report.json" ]; then bash "$ENV" bash "$A/post.sh" > "$A/post_wrapper.log" 2>&1; post=$?; else post=2; fi
if [ "$rc" -ne 0 ]; then exit "$rc"; fi
exit "$post"

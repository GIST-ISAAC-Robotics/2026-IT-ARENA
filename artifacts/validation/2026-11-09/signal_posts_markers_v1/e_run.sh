#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-09/signal_posts_markers_v1
if [[ -e "$A/e_all_posts_blank" || -e "$A/e1.exit" ]]; then exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/e_preflight.txt"; then exit 3; fi
sha256sum src/arena_bringup/launch/simulation.launch.py src/arena_autonomy/arena_autonomy/local_follow.py src/arena_autonomy/arena_autonomy/local_path.py scripts/validate_basic_autonomy.py "$A/marker_posts_provisional.sdf" "$A/e_run.py" > "$A/e_sources_before.txt"
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B "$A/e_run.py" > "$A/e1.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/e1.exit"
sha256sum -c "$A/e_sources_before.txt" > "$A/e_sources_after.txt"
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/e_postflight.txt"
exit "$code"

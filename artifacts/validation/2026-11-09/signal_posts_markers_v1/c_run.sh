#!/usr/bin/env bash
# C 대표 1회: r18 정지차 + 신호 양지주. 각 단계 실제 종료 코드를 남긴다(WSL).
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-09/signal_posts_markers_v1; D=$A/c_stationary_posts
R18=artifacts/validation/2026-11-08/rear_face_motion_connected_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [[ -e "$D/run1" || -e "$D/run1.exit" ]]; then echo 'already run'; exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/preflight_processes_run1.txt"; then echo "PREFLIGHT_BUSY=1" > "$D/run1.exit"; exit 3; fi
sha256sum src/arena_bringup/launch/simulation.launch.py src/arena_autonomy/arena_autonomy/local_follow.py src/arena_autonomy/arena_autonomy/local_path.py scripts/validate_g1_traffic.py > "$D/sources_before.txt"
bash "$ENV" /usr/bin/python3 -B "$A/c_run.py" > "$D/run1_console.log" 2>&1
harness=$?
surface=-1
if [[ -f "$D/run1/report.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$D/run1" --output "$D/surface_run1.json" > "$D/surface_run1.log" 2>&1; surface=$?
fi
goal=-1
if [[ -f "$D/surface_run1.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B -c "import sys; sys.path.insert(0,'$R18'); from goal_eval import evaluate; from pathlib import Path; evaluate(Path('$D/run1').resolve(), Path('src/arena_gazebo/worlds/it_arena_official_v2026_11_06').resolve())" > "$D/goal_run1.log" 2>&1; goal=$?
fi
sha256sum -c "$D/sources_before.txt" > "$D/sources_after.txt" 2>&1; integrity=$?
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/postflight_processes_run1.txt"; leftover=$?
printf 'HARNESS_EXIT=%s\nSURFACE_EXIT=%s\nGOAL_EXIT=%s\nINTEGRITY_EXIT=%s\nLEFTOVER_PGREP=%s\n' "$harness" "$surface" "$goal" "$integrity" "$leftover" > "$D/run1.exit"

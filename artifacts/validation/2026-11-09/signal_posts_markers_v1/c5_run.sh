#!/usr/bin/env bash
# C 대표 1회: r18 정지차 + 신호 양지주. 각 단계 실제 종료 코드를 남긴다(WSL).
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-09/signal_posts_markers_v1; D=$A/c_stationary_posts
R18=artifacts/validation/2026-11-08/rear_face_motion_connected_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [[ -e "$D/run5" || -e "$D/run5.exit" ]]; then echo 'already run'; exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/preflight_processes_run5.txt"; then echo "PREFLIGHT_BUSY=1" > "$D/run5.exit"; exit 3; fi
sha256sum src/arena_bringup/launch/simulation.launch.py src/arena_autonomy/arena_autonomy/local_follow.py src/arena_autonomy/arena_autonomy/local_path.py src/arena_autonomy/arena_autonomy/bypass_handoff_corridor_trial.py src/arena_autonomy/arena_autonomy/nearby_face_motion.py src/arena_autonomy/arena_autonomy/bypass_wall_fit.py scripts/validate_g1_traffic.py > "$D/sources_before_run5.txt"
sha256sum "$A/c5_run.py" "$A/c5_run.sh" "$A/stationary_goal_eval.py" "$A/evaluate_traffic_posts.py" "$A/stationary_posts_assess.py" >> "$D/sources_before_run5.txt"
bash "$ENV" /usr/bin/python3 -B "$A/c5_run.py" > "$D/run5_console.log" 2>&1
harness=$?
surface=-1
if [[ -f "$D/run5/report.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$D/run5" --output "$D/surface_run5.json" > "$D/surface_run5.log" 2>&1; surface=$?
fi
goal=-1
if [[ -f "$D/surface_run5.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B "$A/stationary_goal_eval.py" "$D/run5" src/arena_gazebo/worlds/it_arena_official_v2026_11_06 > "$D/goal_run5.log" 2>&1; goal=$?
fi
bash "$ENV" /usr/bin/python3 -B "$A/evaluate_traffic_posts.py" "$D/run5" > "$D/posts_run5.log" 2>&1; posts=$?
sha256sum -c "$D/sources_before_run5.txt" > "$D/sources_after_run5.txt" 2>&1; integrity=$?
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/postflight_processes_run5.txt"; leftover=$?
assessment=-1
if [[ -f "$D/run5/dense_post_clearance.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B "$A/stationary_posts_assess.py" "$D/run5" > "$D/assessment_run5.log" 2>&1; assessment=$?
fi
printf 'HARNESS_EXIT=%s\nSURFACE_EXIT=%s\nGOAL_EXIT=%s\nPOSTS_EXIT=%s\nASSESSMENT_EXIT=%s\nINTEGRITY_EXIT=%s\nLEFTOVER_PGREP=%s\n' "$harness" "$surface" "$goal" "$posts" "$assessment" "$integrity" "$leftover" > "$D/run5.exit"

exit "$harness"

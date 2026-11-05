#!/usr/bin/env bash
# 등록된 r8 연속 전진율 ON 폐루프 비교 1회 실행과 회수. 재등록·자동 재시도 없음. prepare.py --run만 호출한다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.."
A=artifacts/validation/2026-11-05/moving_follow_continuity_v1/closed_loop_v1
N=run1
if [ -e "$A/$N" ] || [ -e "$A/$N.exit" ]; then echo "existing $N output; no rerun" >&2; exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${N}_preflight.log"; then
  echo "ACTUAL_EXIT_CODE=3 (another simulation is running; run not started)" > "$A/$N.exit"; exit 3
fi
FILES=("$A/prepare.py" "$A/registered.json" "$A/review.py" "$A/run.sh" "$A/../compare.py" "$A/../baseline_metrics.json"
  artifacts/validation/2026-11-05/moving_pass_baseline_v1/course.json artifacts/validation/2026-11-05/moving_pass_baseline_v1/registered.json
  config/tests/official_forward_v1.json config/tests/continuous_forward_pace_r8_sha256.json config/tests/lidar_mount_structure_gate_r7_sha256.json
  src/arena_description/config/b0183_c1_jr60p20_sim.yaml src/arena_gazebo/worlds/it_arena_official/scene.json
  scripts/validate_g1_traffic.py scripts/g1_handoff_eval.py scripts/surface_contact_eval.py
  src/arena_bringup/launch/*.launch.py src/arena_autonomy/arena_autonomy/*.py)
sha256sum "${FILES[@]}" > "$A/${N}_inputs.sha256" || { echo "ACTUAL_EXIT_CODE=4 (input hash failed; run not started)" > "$A/$N.exit"; exit 4; }
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
start=$(date +%s)
echo "start_utc=$(date -u +%FT%TZ)" >> "$A/${N}_preflight.log"
bash "$ENV" /usr/bin/python3 -B "$A/prepare.py" --run > "$A/${N}_console.log" 2>&1
code=$?
wall=$(( $(date +%s) - start ))
echo "end_utc=$(date -u +%FT%TZ)" >> "$A/${N}_preflight.log"
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
s=-1
if [ -d "$A/$N" ]; then
  python3 -B scripts/surface_contact_eval.py "$A/$N" --output "$A/${N}_surface.json" > "$A/${N}_post.log" 2>&1; s=$?
fi
sha256sum -c "$A/${N}_inputs.sha256" >> "$A/${N}_post.log" 2>&1; integrity=$?
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${N}_postflight.log"; left=$?
echo "ACTUAL_EXIT_CODE=$code SURFACE=$s INTEGRITY=$integrity LEFTOVER_PGREP=$left WALL_S=$wall" > "$A/$N.exit"
exit "$code"

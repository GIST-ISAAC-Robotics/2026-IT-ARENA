#!/usr/bin/env bash
# 등록 → 단독 1랩 → 회수. 인자: 실행 이름(기본 run1). 자동 재시도 없음.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-03/solo_post_free_v1
N=${1:-run1}
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${N}_preflight.log"; then
  echo "ACTUAL_EXIT_CODE=3 (another simulation is running)" > "$A/${N}_once.exit"
  exit 3
fi
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
sha256sum install/arena_gazebo/share/arena_gazebo/worlds/it_arena_official/world.sdf src/arena_gazebo/worlds/it_arena_official/world.sdf install/arena_gazebo/lib/arena_gazebo/traffic_light_controller.py src/arena_gazebo/scripts/traffic_light_controller.py >> "$A/${N}_preflight.log"
/usr/bin/python3 -B "$A/run.py" --attempt "$N" --prepare-only >> "$A/${N}_preflight.log" 2>&1 || { echo "ACTUAL_EXIT_CODE=4 (prepare failed)" > "$A/${N}_once.exit"; exit 4; }
/usr/bin/python3 -B "$A/run.py" --attempt "$N"
run=$?
bash "$A/post.sh" "$N" > "$A/${N}_post_steps.log" 2>&1
post=$?
echo "ACTUAL_EXIT_CODE=$run POST_EXIT_CODE=$post" > "$A/${N}_once.exit"
exit "$run"

#!/usr/bin/env bash
# 대표 교통 장면 1회와 회수. 인자: 장면, 실행 이름. 자동 재시도 없음.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_traffic_v1
CASE=$1
NAME=$2
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${NAME}_preflight.log"; then
  echo "ACTUAL_EXIT_CODE=3 (another simulation is running)" > "$A/$NAME.exit"
  exit 3
fi
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" /usr/bin/python3 -B scripts/validate_g1_traffic.py --mode g1g2 --handoff-reserve paced --handoff-yield-progress \
  --handoff-rear-sweep --official-signal-course --config "$PWD/config/tests/official_traffic_v1.json" --case "$CASE" --seed 0 \
  --record-follow-decisions --handoff-observed-motion --output "$PWD/$A/$NAME" > "$A/${NAME}_console.log" 2>&1
code=$?
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B scripts/surface_contact_eval.py "$A/$NAME" --output "$A/${NAME}_surface.json" > "$A/${NAME}_post.log" 2>&1
python3 -I -B "$A/analyze.py" "$NAME" >> "$A/${NAME}_post.log" 2>&1 || true
python3 -B "$A/goal_eval.py" "$A/$NAME" src/arena_gazebo/worlds/it_arena_official >> "$A/${NAME}_post.log" 2>&1
echo "ACTUAL_EXIT_CODE=$code GOAL_EVAL=$?" > "$A/$NAME.exit"
exit "$code"

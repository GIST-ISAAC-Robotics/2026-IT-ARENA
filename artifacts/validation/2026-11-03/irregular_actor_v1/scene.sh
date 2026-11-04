#!/usr/bin/env bash
# 등록한 한 장면만 실행한다. 실패 후 자동 재시도 없음.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-03/irregular_actor_v1
CASE=$1
NAME=$2
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${NAME}_preflight.log"; then
  echo 'ACTUAL_EXIT_CODE=3 (another simulation is running)' > "$A/$NAME.exit"
  exit 3
fi
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" /usr/bin/python3 -B scripts/validate_g1_traffic.py --mode g1g2 --handoff-reserve paced --handoff-yield-progress \
  --handoff-rear-sweep --official-signal-course --config "$PWD/config/tests/irregular_actor_v1.json" --case "$CASE" --seed 0 \
  --record-follow-decisions --handoff-observed-motion --output "$PWD/$A/$NAME" > "$A/${NAME}_console.log" 2>&1
code=$?
bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$A/$NAME" --output "$A/${NAME}_surface.json" > "$A/${NAME}_post.log" 2>&1
surface=$?
bash "$ENV" /usr/bin/python3 -B "$A/analyze.py" "$NAME" >> "$A/${NAME}_post.log" 2>&1
analysis=$?
bash "$ENV" /usr/bin/python3 -B "$A/goal_eval.py" "$A/$NAME" src/arena_gazebo/worlds/it_arena_official >> "$A/${NAME}_post.log" 2>&1
goal=$?
echo "ACTUAL_EXIT_CODE=$code SURFACE_EXIT=$surface ANALYSIS_EXIT=$analysis GOAL_EXIT=$goal" > "$A/$NAME.exit"
exit "$code"

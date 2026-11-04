#!/usr/bin/env bash
# 대표 전방 장면 1회. 인자: 장면 이름, 실행 폴더 이름. 자동 재시도 없음.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_forward_v1
CASE=$1
NAME=$2
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${NAME}_preflight.log"; then
  echo "ACTUAL_EXIT_CODE=3 (another simulation is running)" > "$A/$NAME.exit"
  exit 3
fi
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" /usr/bin/python3 -B scripts/validate_g1_traffic.py --mode g1g2 --handoff-reserve paced --handoff-yield-progress \
  --handoff-rear-sweep --official-signal-course --config "$PWD/config/tests/official_forward_v1.json" --case "$CASE" --seed 0 \
  --record-follow-decisions ${3:-} --output "$PWD/$A/$NAME" > "$A/${NAME}_console.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/$NAME.exit"
exit "$code"

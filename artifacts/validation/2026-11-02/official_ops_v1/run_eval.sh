#!/usr/bin/env bash
# 대표 운영 흐름 1회. 인자: 출력 이름, 나머지는 평가기 인자. 자동 재시도 없음.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_ops_v1
NAME=$1; shift
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${NAME}_preflight.log"; then
  echo "ACTUAL_EXIT_CODE=3 (another simulation is running)" > "$A/$NAME.exit"; exit 3
fi
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" /usr/bin/python3 -B scripts/race_ops_eval.py --output "$PWD/$A/$NAME" "$@" > "$A/${NAME}_console.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/$NAME.exit"
exit "$code"

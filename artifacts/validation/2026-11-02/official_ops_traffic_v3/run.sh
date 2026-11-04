#!/usr/bin/env bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_ops_traffic_v3
NAME=${1:?name required}
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${NAME}_preflight.log"; then
  echo 'ACTUAL_EXIT_CODE=3 (another simulation running)' > "$A/$NAME.exit"
  exit 3
fi
set +e
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B scripts/race_ops_eval.py \
  --output "$PWD/$A/$NAME" --scenario parked_recovery --first-stop-s 0.5 --browser-stop > "$A/${NAME}_console.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/$NAME.exit"
exit "$code"

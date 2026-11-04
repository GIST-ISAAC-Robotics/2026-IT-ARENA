#!/usr/bin/env bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_ops_recovery_v2
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/d1_preflight.log"; then
  echo 'ACTUAL_EXIT_CODE=3 (another simulation running)' > "$A/d1.exit"
  exit 3
fi
set +e
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B scripts/race_ops_eval.py \
  --output "$PWD/$A/d1" --scenario across_marker > "$A/d1_console.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/d1.exit"
exit "$code"

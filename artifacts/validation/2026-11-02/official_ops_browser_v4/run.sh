#!/usr/bin/env bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_ops_browser_v4
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/b1_preflight.log"; then
  echo 'ACTUAL_EXIT_CODE=3 (another simulation running)' > "$A/b1.exit"
  exit 3
fi
set +e
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B scripts/race_ops_eval.py \
  --output "$PWD/$A/b1" --scenario browser_stop > "$A/b1_console.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/b1.exit"
exit "$code"

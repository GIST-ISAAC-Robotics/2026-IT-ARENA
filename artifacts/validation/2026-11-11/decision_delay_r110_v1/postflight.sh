#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
python3 -B artifacts/validation/2026-11-10/facility_traffic_stability_v1/analyze_continuity.py artifacts/validation/2026-11-11/decision_delay_r110_v1/two_actor_run1/run1 --output artifacts/validation/2026-11-11/decision_delay_r110_v1/two_actor_run1/run1/continuity.json > build/r110_continuity.log 2>&1
set +e
python3 -B artifacts/validation/2026-11-11/decision_delay_r110_v1/assess.py > build/r110_assess.log 2>&1
assessment_status=$?
set -e
python3 -B artifacts/validation/2026-11-11/decision_delay_r110_v1/analyze_delivery.py > build/r110_delivery_analysis.log 2>&1
pgrep -af 'arena_autonomy_test_989|decision_delay_r110_v1/two_actor_run1|decision_delay_r110_v1/node_entry.py' | grep -v 'pgrep -af' > artifacts/validation/2026-11-11/decision_delay_r110_v1/two_actor_run1/postflight_processes.txt || true
exit "$assessment_status"

#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
/usr/bin/python3 -B artifacts/validation/2026-11-10/facility_traffic_stability_v1/analyze_continuity.py artifacts/validation/2026-11-12/guard_current_r143_v1/two_actor_run1/run1 --output artifacts/validation/2026-11-12/guard_current_r143_v1/two_actor_run1/run1/continuity.json > build/r143_continuity.log 2>&1
pgrep -af 'arena_autonomy_test_989|guard_recheck_r139_v1/two_actor_run1|guard_current_r143_v1/node_entry.py' | grep -v 'pgrep -af' > artifacts/validation/2026-11-12/guard_current_r143_v1/two_actor_run1/postflight_processes.txt || true
/usr/bin/python3 -B scripts/surface_contact_eval.py artifacts/validation/2026-11-12/guard_current_r143_v1/two_actor_run1/run1 --output artifacts/validation/2026-11-12/guard_current_r143_v1/two_actor_run1/surface.json > build/r143_surface.log 2>&1
set +e
/usr/bin/python3 -B artifacts/validation/2026-11-12/guard_current_r143_v1/assess.py > build/r143_assess.log 2>&1
assessment_status=$?
set -e
/usr/bin/python3 -B artifacts/validation/2026-11-12/guard_current_r143_v1/analyze_delivery.py > build/r143_delivery_analysis.log 2>&1
exit "$assessment_status"

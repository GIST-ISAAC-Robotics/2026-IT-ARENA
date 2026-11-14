#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q tests/test_guard_memory_command.py tests/test_guard_memory_ray_clearance.py tests/test_independent_guard_route.py tests/test_partial_observation.py tests/test_traffic_recovery_contract.py tests/test_known_structure_policy.py tests/test_guard_observation_policy.py tests/test_guard_observation_rebase.py tests/test_runtime_guard_crawl.py tests/test_current_release_wait.py tests/test_current_wall_reacquire.py tests/test_local_follow.py tests/test_local_follow_validation.py tests/test_moving_pass.py tests/test_current_route_proof.py tests/test_fresh_current_delivery.py tests/test_motion_ingress_budget.py tests/test_motion_ingress_drain.py tests/test_compact_follow_trace.py tests/test_runtime_plan_trace.py --tb=short --junitxml=artifacts/validation/2026-11-14/guard_memory_ray_clearance_r532_v1/common_transfer/regression.xml

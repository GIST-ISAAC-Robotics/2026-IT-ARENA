#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q tests/test_r0_plus.py tests/test_r0_plus_integration.py tests/test_local_follow.py tests/test_follow_trace.py tests/test_follow_profile.py tests/test_g1_traffic_eval.py tests/test_g1_solo_candidate_validation.py tests/test_traffic_actors.py tests/test_traffic_actor_runtime.py tests/test_traffic_actor_integration.py

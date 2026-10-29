#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/bump_traffic_v1
test ! -e "$base/tail_contracts_v1.log"
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_actor_tail_drain.py tests/test_actor_tail_provenance.py tests/test_rotation_reobservation_provenance.py tests/test_source_contract_helpers.py tests/test_source_provenance_r0_plus_b3.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py "$base/test_trial.py" "$base/test_analysis.py" > "$base/tail_contracts_v1.log" 2>&1

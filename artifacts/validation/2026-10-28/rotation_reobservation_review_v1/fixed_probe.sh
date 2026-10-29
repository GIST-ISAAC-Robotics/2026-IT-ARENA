#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/rotation_reobservation_review_v1
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_consistent.py" > "$base/fixed_contracts_v1.log" 2>&1
for name in run3 run2; do
  /usr/bin/python3 -B "$base/probe_candidate.py" "$base/$name/fixture.json" --variant fixed --output "$base/$name/fixed_candidate_v1.json" > "$base/$name/fixed_candidate_v1.log" 2>&1
done

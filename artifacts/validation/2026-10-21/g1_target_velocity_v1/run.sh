#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
out=artifacts/validation/2026-10-21/g1_target_velocity_v1
if [ -e "$out/pytest.log" ]; then exit 2; fi
set +e
python3 -B -m pytest -q -p no:cacheprovider tests/test_observation_target_extent.py tests/test_target_velocity_bounds.py > "$out/pytest.log" 2>&1
code=$?
printf '\nPYTEST_EXIT_CODE=%s\n' "$code" >> "$out/pytest.log"
if [ "$code" -ne 0 ]; then tail -n 40 "$out/pytest.log"; exit "$code"; fi
python3 -B scripts/probe_target_velocity_bounds.py --output "$out/run" > "$out/probe.log" 2>&1
code=$?
printf '\nPROBE_EXIT_CODE=%s\n' "$code" >> "$out/probe.log"
tail -n 4 "$out/pytest.log"
tail -n 6 "$out/probe.log"
exit "$code"

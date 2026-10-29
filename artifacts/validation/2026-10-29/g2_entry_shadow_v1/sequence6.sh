#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_entry_shadow_v1
test ! -e "$base/sequence6_tests.log" || exit 90
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_bypass_entry_trial.py tests/test_bypass_entry_sequence.py tests/test_bypass_entry_guide.py > "$base/sequence6_tests.log" 2>&1
tests=$?
if test "$tests" -ne 0; then
  printf '{"tests":%s,"probe":null}\n' "$tests" > "$base/sequence6_exit.json"
  exit "$tests"
fi
/usr/bin/python3 -B scripts/probe_bypass_entry_sequence.py --output "$base/sequence6" --duration 8 --guide > "$base/sequence6_probe.log" 2>&1
probe=$?
printf '{"tests":%s,"probe":%s}\n' "$tests" "$probe" > "$base/sequence6_exit.json"
exit "$probe"

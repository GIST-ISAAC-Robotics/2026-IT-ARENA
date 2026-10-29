#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_entry_shadow_v1
test ! -e "$base/motion19_probe.log" || exit 90
/usr/bin/python3 -B -c "import ackermann_msgs.msg" || exit 93
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_bypass_entry_trial.py tests/test_bypass_entry_sequence.py tests/test_bypass_entry_guide.py tests/test_bypass_probe_sensors.py tests/test_bypass_wall_reanchor.py tests/test_bypass_wall_fit.py > "$base/motion19_tests.log" 2>&1
tests=$?
if test "$tests" -ne 0; then
  printf '{"tests":%s,"probe":null}\n' "$tests" > "$base/motion19_exit.json"
  exit "$tests"
fi
/usr/bin/python3 -B scripts/probe_bypass_entry_sequence.py --output "$base/motion19" --duration 10 --guide --stationary-outer-boundary --wide-entry --rejoin --wall-reanchor --tracked-wall-fit --sensor-profile motion > "$base/motion19_probe.log" 2>&1
probe=$?
printf '{"tests":%s,"probe":%s}\n' "$tests" "$probe" > "$base/motion19_exit.json"
exit "$probe"

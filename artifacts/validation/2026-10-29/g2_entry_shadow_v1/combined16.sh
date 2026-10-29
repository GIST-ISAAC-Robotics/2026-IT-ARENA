#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_entry_shadow_v1
test ! -e "$base/combined16_probe.log" || exit 90
/usr/bin/python3 -B -c "import ackermann_msgs.msg" || exit 93
tests='"reused_motion15_same_sources"'
/usr/bin/python3 -B scripts/probe_bypass_entry_sequence.py --output "$base/combined16" --duration 10 --guide --stationary-outer-boundary --wide-entry --rejoin --sensor-profile combined > "$base/combined16_probe.log" 2>&1
probe=$?
printf '{"tests":%s,"probe":%s}\n' "$tests" "$probe" > "$base/combined16_exit.json"
exit "$probe"

#!/usr/bin/env bash
set -o pipefail
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_entry_shadow_v1
if test -e "$base/motion14_retry_probe.log" || test -e "$base/motion14_retry_exit.json" || test -e "$base/motion14_retry"; then exit 90; fi
/usr/bin/python3 -B -c "import ackermann_msgs" > "$base/motion14_retry_import.log" 2>&1
imp=$?
if test "$imp" -ne 0; then printf "{\"import\":%s,\"probe\":null}\n" "$imp" > "$base/motion14_retry_exit.json"; exit "$imp"; fi
/usr/bin/python3 -B scripts/probe_bypass_entry_sequence.py --output "$base/motion14_retry" --duration 10 --guide --stationary-outer-boundary --wide-entry --rejoin --sensor-profile motion > "$base/motion14_retry_probe.log" 2>&1
probe=$?
printf "{\"import\":%s,\"probe\":%s}\n" "$imp" "$probe" > "$base/motion14_retry_exit.json"
exit "$probe"

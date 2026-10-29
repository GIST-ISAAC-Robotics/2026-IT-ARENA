#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_entry_shadow_v1
test ! -e "$base/sequence9_probe.log" || exit 90
tests='"reused_sequence8_same_sources"'
/usr/bin/python3 -B scripts/probe_bypass_entry_sequence.py --output "$base/sequence9" --duration 8 --guide --stationary-outer-boundary --wide-entry > "$base/sequence9_probe.log" 2>&1
probe=$?
printf '{"tests":%s,"probe":%s}\n' "$tests" "$probe" > "$base/sequence9_exit.json"
exit "$probe"

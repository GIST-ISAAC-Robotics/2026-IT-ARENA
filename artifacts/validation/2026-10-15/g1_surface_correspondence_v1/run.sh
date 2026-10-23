#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_surface_correspondence_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_surface_correspondence.py tests/test_observed_fragments.py tests/test_fragment_links.py > "$base/tests.log" 2>&1
python3 -B scripts/probe_surface_correspondence.py artifacts/validation/2026-10-15/g1_solo_lap_v4/run/follow_decisions.jsonl --output "$base/probe" > "$base/probe.log" 2>&1
tail -3 "$base/tests.log"
tail -1 "$base/probe.log"

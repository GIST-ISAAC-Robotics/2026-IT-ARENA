#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
case "$1" in initial_occlusion|first_curve) ;; *) exit 2 ;; esac
out="artifacts/validation/2026-10-07/g1b_extensions_v1/$1_s0"
test ! -e "$out"
test ! -e "$out.runner.log"
python3 -B scripts/validate_g1_traffic.py --case "$1" --seed 0 --mode g1 --config config/tests/g1_traffic_extensions_v1.json --output "$out" --record-braking-diagnostics --record-wheel-contacts > "$out.runner.log" 2>&1

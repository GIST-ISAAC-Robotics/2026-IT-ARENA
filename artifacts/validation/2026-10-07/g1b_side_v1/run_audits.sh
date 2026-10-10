#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
base='artifacts/validation/2026-10-07/g1b_side_v1'
python3 -B scripts/audit_g1_traffic_coverage.py "$base/side_left_s0" --output "$base/side_left_postcoverage.json" > "$base/side_left_postcoverage.log" 2>&1
python3 -B scripts/audit_g1_traffic_sources.py "$base/side_left_s0" --freeze "$base/source_freeze.json" --postcoverage "$base/side_left_postcoverage.json" --output "$base/side_left_source_audit.json" > "$base/side_left_source_audit.log" 2>&1
python3 -B scripts/audit_motor_encoder_runtime.py "$base/side_left_s0" --output "$base/side_left_encoder.json" > "$base/side_left_encoder.log" 2>&1

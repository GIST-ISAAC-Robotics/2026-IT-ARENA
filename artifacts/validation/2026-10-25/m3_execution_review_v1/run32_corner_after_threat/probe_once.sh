#!/usr/bin/env bash
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests"
python3 -B artifacts/validation/2026-10-25/m3_execution_review_v1/probe_straight_surface_horizon.py artifacts/validation/2026-10-25/m3_execution_review_v1/run32_corner_after_threat/run/follow_decisions.jsonl artifacts/validation/2026-10-25/m3_execution_review_v1/run32_corner_after_threat/straight_surface_horizon.json

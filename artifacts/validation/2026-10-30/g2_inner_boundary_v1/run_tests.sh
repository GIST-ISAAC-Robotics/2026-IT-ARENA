#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 -B artifacts/validation/2026-10-30/g2_inner_boundary_v1/test_candidate.py

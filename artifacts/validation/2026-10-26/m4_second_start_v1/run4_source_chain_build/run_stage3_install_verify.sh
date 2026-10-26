#!/usr/bin/env bash
set -o pipefail
repo="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
cd "$repo" || exit 90
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
printf "PYTHONPATH_AFTER_SETUP=%s\n" "${PYTHONPATH:-}"
python3 "$repo/artifacts/validation/2026-10-26/m4_second_start_v1/run4_source_chain_build/verify_install.py" "$repo/artifacts/validation/2026-10-26/m4_second_start_v1/run4_source_chain_build"

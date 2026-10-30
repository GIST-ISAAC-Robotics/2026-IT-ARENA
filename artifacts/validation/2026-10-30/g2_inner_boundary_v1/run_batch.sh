#!/usr/bin/env bash
# 사용: run_batch.sh <batch 이름>
set -eo pipefail
batch="$1"
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/g2_inner_boundary_v1
test ! -e "$base/$batch.log"
/usr/bin/python3 -B "$base/test_candidate.py" > "$base/$batch.tests.log" 2>&1
set +e
/usr/bin/python3 -B "$base/$batch.py" > "$base/$batch.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/$batch.exit"
exit "$rc"

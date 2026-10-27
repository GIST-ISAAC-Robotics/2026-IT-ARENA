#!/usr/bin/env bash
# 사용: run_pytest.sh <log> <tests...>. 기존 log는 덮어쓰지 않는다.
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
log=$1; shift
test ! -e "$log"
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
set +e
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$@" > "$log" 2>&1
rc=$?
set -e
echo "exit=$rc" >> "$log"
exit $rc

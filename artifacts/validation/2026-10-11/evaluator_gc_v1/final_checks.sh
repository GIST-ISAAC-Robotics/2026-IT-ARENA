#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash || exit
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit
source install/local_setup.bash || exit
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-11/evaluator_gc_v1
tag="${1:-final}"
case "$tag" in final|final_v2) ;; *) exit 2 ;; esac
test ! -e "$base/pytest_$tag.log" || exit 2
python3 -B -m pytest -q -p no:cacheprovider tests > "$base/pytest_$tag.log" 2>&1
rc=$?
tail -8 "$base/pytest_$tag.log"
exit "$rc"

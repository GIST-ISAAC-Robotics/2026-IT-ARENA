#!/usr/bin/env bash
# 이번 변경 전 HEAD 사본에서 같은 실패 항목만 재실행해 기존/신규를 구분한다.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$HERE/../../../.."
B="$HOME/arena_scratch/base_59c3c3b0"
rm -rf "$B"; mkdir -p "$B"
git -C "$REPO" archive 59c3c3b0 | tar -x -C "$B"
cd "$B"
set +u; source /opt/ros/jazzy/setup.bash; source "$REPO/install/local_setup.bash"
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
grep '^FAILED ' "$HERE/tests_full1.log" | sed 's/^FAILED //; s/ - .*$//' > "$HERE/failed_nodes.txt"
python3 -B -m pytest -q -p no:cacheprovider --tb=line $(cat "$HERE/failed_nodes.txt" | tr '\n' ' ') > "$HERE/baseline_failed.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$HERE/baseline_failed.exit"
exit 0

#!/usr/bin/env bash
# 10/19 재시작 명령 경로 진단·시험기 수정 최종본의 전체 회귀 1회.
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
ART="$ROOT/artifacts/validation/2026-10-19/restart_command_path_v1/regression_full"
cd "$ROOT"
mkdir -p "$ART"
printf '%s\n' 'python3 -B -m pytest -q -p no:cacheprovider tests' > "$ART/command.txt"
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ART/source_hashes_before.sha256"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
set +e
python3 -B -m pytest -q -p no:cacheprovider tests > "$ART/pytest.log" 2>&1
status=$?
set -e
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ART/source_hashes_after.sha256"
if diff -u "$ART/source_hashes_before.sha256" "$ART/source_hashes_after.sha256" > "$ART/source_hash_diff.txt"; then echo SOURCE_UNCHANGED > "$ART/source_hash_status.txt"; else echo SOURCE_CHANGED > "$ART/source_hash_status.txt"; fi
ps -eo pid,ppid,stat,etimes,args | grep -E "pytest|validate_pipeline|arena_" | grep -v grep > "$ART/process_after.txt" || true
printf 'EXIT_STATUS=%s\n' "$status" > "$ART/runner_status.txt"
tail -3 "$ART/pytest.log"
exit "$status"

#!/usr/bin/env bash
# 사용: scratch_check.sh <label> <tests...>
# 작업트리를 바꾸지 않고 ~/arena_scratch 사본에 patched/ 파일을 덮어 정적·단위 검사를 한다.
# 진행 중인 Gazebo 주행을 방해하지 않도록 nice 19, 시뮬레이터·빌드 없음.
set -o pipefail
REPO='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A="$REPO/artifacts/validation/2026-11-07/traffic_course_migration_v1"
S="$HOME/arena_scratch/traffic_course_migration_v1"
label="$1"; shift
mkdir -p "$A/checks"
rm -rf "$S"; mkdir -p "$S"
for d in scripts tests config src; do
  rsync -a --exclude '__pycache__' --exclude 'build' --exclude 'install' --exclude 'log' "$REPO/$d" "$S/"
done
cp "$A"/patched/scripts/*.py "$S/scripts/"
cp "$A"/patched/tests/*.py "$S/tests/"
cd "$S" || exit 2
set +u; source /opt/ros/jazzy/setup.bash; set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$S/src/arena_autonomy:$S/src/arena_vehicle_interface:$S/scripts:$S/tests"
out="$A/checks/${label}"
{
  echo "# base sha256 (working tree at check time)"
  (cd "$REPO" && sha256sum scripts/validate_g1_traffic.py scripts/official_traffic_course.py scripts/race_ops_eval.py scripts/summarize_motion_lap.py)
  echo "# patched sha256"
  (cd "$A/patched" && sha256sum scripts/*.py tests/*.py)
  python3 -B -m py_compile scripts/validate_g1_traffic.py scripts/official_traffic_course.py scripts/race_ops_eval.py scripts/summarize_motion_lap.py tests/test_traffic_course_track.py && echo PY_COMPILE_OK
  nice -n 19 python3 -B -m pytest -q -p no:cacheprovider --tb=short "$@"
} > "${out}.log" 2>&1
result=$?
echo "ACTUAL_EXIT_CODE=$result" > "${out}.exit"
exit "$result"

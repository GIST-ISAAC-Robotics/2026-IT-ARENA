#!/usr/bin/env bash
# 사용: run_tests.sh <label> <real|patched> <test files...>
# WSL ROS 환경에서 pytest만 실행한다(시뮬레이터·빌드 없음, python3 -B로 pyc 미생성).
# patched이면 이 폴더의 patched/를 PYTHONPATH 맨 앞에 두어 validate_g1_traffic만 staged 사본으로 바꾼다.
# 나머지 모듈(validate_basic_autonomy 등)과 REPO는 실제 저장소의 scripts/에서 온다.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
STAGE=artifacts/validation/2026-11-08/g1g2_stress_staging_v1
label="$1"; target="$2"; shift 2
case "$target" in
  real) front=""; export HARNESS_EXPECTED_FILE="$PWD/scripts/validate_g1_traffic.py" ;;
  patched) front="$PWD/$STAGE/patched:"; export HARNESS_EXPECTED_FILE="$PWD/$STAGE/patched/validate_g1_traffic.py" ;;
  mutant_*) front="$PWD/$STAGE/checks/$target:"; export HARNESS_EXPECTED_FILE="$PWD/$STAGE/checks/$target/validate_g1_traffic.py" ;;
  *) echo "target must be real|patched|mutant_*"; exit 2 ;;
esac
export PYTHONPATH="${front}/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
out=$STAGE/checks/pytest_${label}_${target}
python3 -B -m pytest -q -p no:cacheprovider -rA --tb=short "$@" > "${out}.log" 2>&1
result=$?
echo "ACTUAL_EXIT_CODE=$result" > "${out}.exit"
exit "$result"

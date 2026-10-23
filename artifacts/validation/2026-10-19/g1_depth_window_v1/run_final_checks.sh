#!/usr/bin/env bash
# 사용자 중단 요청에 따른 마지막 경계 회귀만. 비용 측정/주행은 실행하지 않는다.
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
out=artifacts/validation/2026-10-19/g1_depth_window_v1
if [ -e "$out/final_pytest.log" ]; then exit 2; fi
sha256sum src/arena_autonomy/arena_autonomy/observation_support.py "$out/candidate.py" tests/test_observation_depth_window.py > "$out/final_before.sha256"
set +e
python3 -B -m pytest -q -p no:cacheprovider tests/test_observation_depth_window.py tests/test_observation_depth_chunk.py tests/test_observation_depth_equivalence.py > "$out/final_pytest.log" 2>&1
code=$?
printf '\nPYTEST_EXIT_CODE=%s\n' "$code" >> "$out/final_pytest.log"
sha256sum src/arena_autonomy/arena_autonomy/observation_support.py "$out/candidate.py" tests/test_observation_depth_window.py > "$out/final_after.sha256"
diff -u "$out/final_before.sha256" "$out/final_after.sha256" > "$out/final_source_diff.txt"
hash_code=$?
tail -n 30 "$out/final_pytest.log"
if [ "$code" -ne 0 ]; then exit "$code"; fi
exit "$hash_code"

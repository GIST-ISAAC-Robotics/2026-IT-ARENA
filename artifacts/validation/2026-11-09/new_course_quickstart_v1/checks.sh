#!/usr/bin/env bash
# 새 코스 단독 프리셋 정적 검사. 시뮬레이터·녹화는 띄우지 않는다.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
d=artifacts/validation/2026-11-09/new_course_quickstart_v1
python3 scripts/build_official_track.py --check --profile config/tracks/official_v2026.11.06.yaml > $d/track_check.log 2>&1; echo "ACTUAL_EXIT_CODE=$?" > $d/track_check.exit
python3 -B scripts/run_new_course_solo.py /tmp/nc_quickstart_never_created --preset new-course-solo --check-r25 > $d/check_r25.json 2>&1; echo "ACTUAL_EXIT_CODE=$?" > $d/check_r25.exit
python3 -B scripts/run_new_course_solo.py /tmp/nc_quickstart_never_created --preset new-course-solo --print-args > $d/print_args.json 2>&1; echo "ACTUAL_EXIT_CODE=$?" > $d/print_args.exit
test ! -e /tmp/nc_quickstart_never_created; echo "OUTPUT_NOT_CREATED_EXIT=$?" >> $d/print_args.exit
python3 -B -m pytest -q -p no:cacheprovider --tb=short tests/test_new_course_quickstart.py tests/test_b0390_profile.py tests/test_jr60_adoption_profile.py tests/test_solo_corner_approach.py tests/test_solo_straight_budget.py > $d/pytest.log 2>&1; echo "ACTUAL_EXIT_CODE=$?" > $d/pytest.exit

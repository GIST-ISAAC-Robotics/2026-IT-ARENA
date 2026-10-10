#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash || exit
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit
source install/local_setup.bash || exit
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-11/initial_startup_v1
case "${1:?action}" in
  pretests|pretests_r6_v2|finaltests|finaltests_v2)
    test ! -e "$base/pytest_$1.log" || exit 2
    if test "$1" != finaltests && test "$1" != finaltests_v2; then
      python3 -B -m pytest -q -p no:cacheprovider tests/test_evaluation_startup.py tests/test_initial_startup_analysis.py tests/test_evaluation_gc.py tests/test_scheduled_command.py tests/test_physics_step_option.py tests/test_runtime_world_options.py > "$base/pytest_$1.log" 2>&1
    else
      python3 -B -m pytest -q -p no:cacheprovider tests > "$base/pytest_$1.log" 2>&1
    fi
    rc=$?; tail -12 "$base/pytest_$1.log"; exit "$rc" ;;
  s1_service|e1_embedded|e2_embedded)
    python3 -B scripts/run_probe_batch.py --plan "$base/plan_$1.json" --status "$base/status_$1.json" ;;
  *) exit 2 ;;
esac

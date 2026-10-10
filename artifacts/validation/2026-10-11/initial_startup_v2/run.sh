#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash || exit
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit
source install/local_setup.bash || exit
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-11/initial_startup_v2
case "${1:?action}" in
  e3_embedded)
    python3 -B scripts/run_probe_batch.py --plan "$base/plan_e3_embedded.json" --status "$base/status_e3_embedded.json" ;;
  tests)
    test ! -e "$base/pytest_analysis.log" || exit 2
    python3 -B -m pytest -q -p no:cacheprovider tests/test_initial_repeat_analysis.py tests/test_initial_startup_analysis.py tests/test_evaluation_startup.py tests/test_command_trace.py tests/test_scheduled_phase_analysis.py > "$base/pytest_analysis.log" 2>&1
    rc=$?; tail -12 "$base/pytest_analysis.log"; exit "$rc" ;;
  *) exit 2 ;;
esac

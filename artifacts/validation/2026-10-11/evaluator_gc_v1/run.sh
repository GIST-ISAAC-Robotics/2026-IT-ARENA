#!/usr/bin/env bash
# ROS setup이 unset 변수를 참조하므로 nounset은 사용하지 않는다.
source /opt/ros/jazzy/setup.bash || exit
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit
source install/local_setup.bash || exit
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-11/evaluator_gc_v1
case "${1:?case or tests}" in
  tests)
    test ! -e "$base/pytest_pre.log" || exit 2
    python3 -B -m pytest -q -p no:cacheprovider tests/test_evaluation_gc.py tests/test_schedule_gate.py tests/test_command_trace.py tests/test_physics_step_option.py tests/test_gate_audit_v2.py > "$base/pytest_pre.log" 2>&1
    rc=$?; tail -5 "$base/pytest_pre.log"; exit "$rc" ;;
  a1_native|b1_defer|b2_defer|a2_native)
    python3 -B scripts/run_probe_batch.py --plan "$base/plan_$1.json" --status "$base/status_$1.json" ;;
  *) exit 2 ;;
esac

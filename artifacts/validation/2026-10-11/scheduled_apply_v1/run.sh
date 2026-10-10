#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash || exit
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit
source install/local_setup.bash || exit
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-11/scheduled_apply_v1
action=${1:?action}
test ! -e "$base/$action.log" || exit 2
case "$action" in
  pretests*) python3 -B -m pytest -q -p no:cacheprovider tests/test_scheduled_apply.py tests/test_evaluation_startup.py tests/test_actuation_ros_nodes.py tests/test_applied_guard.py > "$base/$action.log" 2>&1 ;;
  build*) colcon build --base-paths src --packages-select arena_gazebo arena_vehicle_interface arena_bringup --symlink-install > "$base/$action.log" 2>&1 ;;
  register) python3 -B "$base/register.py" > "$base/$action.log" 2>&1 ;;
  analysis*) python3 -B scripts/analyze_scheduled_apply.py --base "$PWD/$base" --output "$base/$action.json" > "$base/$action.log" 2>&1 ;;
  finalize) python3 -B "$base/finalize.py" > "$base/$action.log" 2>&1 ;;
  alltests*) python3 -B -m pytest -q -p no:cacheprovider tests > "$base/$action.log" 2>&1 ;;
  n1_native|t1_scheduled|t2_scheduled) python3 -B scripts/run_probe_batch.py --plan "$base/plan_$action.json" --status "$base/status_$action.json" > "$base/$action.log" 2>&1 ;;
  *) exit 2 ;;
esac
rc=$?; tail -20 "$base/$action.log"; exit "$rc"

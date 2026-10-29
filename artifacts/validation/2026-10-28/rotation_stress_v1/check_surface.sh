#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/rotation_stress_v1
test ! -e "$base/surface_candidate_tests_v2.log"
set +e
/usr/bin/python3 -B -c 'import ackermann_msgs,sys; print(sys.executable); print(ackermann_msgs.__file__)' > "$base/surface_ros_environment_v2.log" 2>&1
env_exit=$?
if [ "$env_exit" -eq 0 ]; then
  /usr/bin/python3 -B -m pytest -q tests/test_surface_reobservation.py tests/test_surface_reobservation_connection.py tests/test_partial_observation.py tests/test_surface_anchor_probe.py tests/test_recovery_threat_capture.py tests/test_traffic_recovery_contract.py tests/test_anchor_clearance_probe.py > "$base/surface_candidate_tests_v2.log" 2>&1
  test_exit=$?
else
  test_exit=91
fi
printf 'env=%s\ntests=%s\n' "$env_exit" "$test_exit" > "$base/surface_candidate_tests_v2.exit.txt"
exit "$test_exit"

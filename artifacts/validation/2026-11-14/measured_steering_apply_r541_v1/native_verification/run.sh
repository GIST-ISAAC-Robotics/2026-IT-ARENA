set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p build/validation/r541
c++ -std=c++17 -O2 -I src/arena_vehicle_interface/portable artifacts/validation/2026-11-14/measured_steering_apply_r541_v1/stage/test_measured_steering.cpp -o build/validation/r541/servo_test
build/validation/r541/servo_test
python3 -m pytest -q -p no:cacheprovider tests/test_guard_memory_command.py tests/test_guard_memory_ray_clearance.py tests/test_steering_calibration.py tests/test_steering_transport_limit.py tests/test_steering_zero_profile_cli.py tests/test_runtime_installed_profile.py artifacts/validation/2026-11-14/guard_memory_relevance_r538_v1/review_tests/test_relevance.py --junitxml=artifacts/validation/2026-11-14/measured_steering_apply_r541_v1/native_verification/pytest.xml

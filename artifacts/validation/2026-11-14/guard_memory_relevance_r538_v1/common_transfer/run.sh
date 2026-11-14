set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q -p no:cacheprovider tests/test_guard_memory_command.py tests/test_guard_memory_ray_clearance.py tests/test_independent_guard_route.py tests/test_lidar_safety.py artifacts/validation/2026-11-14/guard_memory_relevance_r538_v1/review_tests/test_relevance.py --junitxml=artifacts/validation/2026-11-14/guard_memory_relevance_r538_v1/common_transfer/pytest.xml

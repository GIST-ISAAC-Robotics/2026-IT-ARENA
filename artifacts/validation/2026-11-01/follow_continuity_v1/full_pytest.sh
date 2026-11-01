set +u
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q -p no:cacheprovider tests > artifacts/validation/2026-11-01/follow_continuity_v1/pytest_full.log 2>&1
printf 'ACTUAL_EXIT_CODE=%s\n' "$?" >> artifacts/validation/2026-11-01/follow_continuity_v1/pytest_full.log

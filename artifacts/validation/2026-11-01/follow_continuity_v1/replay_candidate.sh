set +u
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
cd artifacts/validation/2026-11-01/follow_continuity_v1
python3 -B replay.py "$1" ../rolling_lead_v1/run1 ../rolling_lead_v1/run_recent ../race_follow_closing_v1/stop ../rolling_lead_v1/stop_recent ../race_follow_closing_v1/slow --option low_motion_reserve 2>&1 | cut -c1-900

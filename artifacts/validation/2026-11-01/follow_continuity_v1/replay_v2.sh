set +u
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
cd artifacts/validation/2026-11-01/follow_continuity_v1
python3 -B replay.py candidate_replay_v2.json ../rolling_lead_v1/run1 ../race_follow_closing_v1/stop ../race_follow_closing_v1/slow fast_stop_closing --option low_motion_reserve,moving_pace_floor 2>&1 | cut -c1-900
python3 -B replay.py candidate_self_replay_v1_after_v2.json ../rolling_lead_v1/curve_low_motion ../rolling_lead_v1/stop_low_motion fast_stop_low_motion 2>&1 | cut -c1-120
printf 'ACTUAL_EXIT_CODE=%s\n' "$?"

#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_traffic_v1
O=artifacts/validation/2026-11-02/official_forward_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
rm -f "$A/a4_entry_replay.json"
python3 -B "$A/replay_entry.py" "$O/a4" "$A/a4_entry_replay.json"
echo "EXIT=$?"

#!/usr/bin/env bash
# 가상 시점 후보 렌더 2회(signal, markers). 주행 없음. 실패해도 재시도하지 않는다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_start_roi_v1
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/render_preflight.log"; then
  echo "ACTUAL_EXIT_CODE=3 (another simulation is running)" > "$A/render.exit"
  exit 3
fi
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -B "$A/candidate_render.py" signal > "$A/render_signal.log" 2>&1
a=$?
/usr/bin/python3 -B "$A/candidate_render.py" markers > "$A/render_markers.log" 2>&1
b=$?
echo "SIGNAL_EXIT_CODE=$a MARKERS_EXIT_CODE=$b" > "$A/render.exit"
exit 0

#!/usr/bin/env bash
# v2026.11.06 파생 월드 실제 점등·RGB 판독 1회(11/2 render.sh 사본). 인자: 출력 폴더 이름.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-07/track_migration_v1
NAME=${1:-render_v1}
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -B "$A/live_signal_render.py" --output "$A/$NAME" > "$A/$NAME.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/$NAME.exit"
exit "$code"

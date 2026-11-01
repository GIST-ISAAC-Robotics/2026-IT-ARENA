#!/usr/bin/env bash
# 공통 평가 환경. 호출: bash env.sh <명령...>
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests:$PYTHONPATH"
export PYTHONDONTWRITEBYTECODE=1
exec "$@"

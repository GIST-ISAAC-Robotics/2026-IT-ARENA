#!/usr/bin/env bash
# 대역 차량 + 실제 어댑터. Windows Chrome이 localhost로 접속한다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_ops_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
echo stub-token-12345 > "$A/stub_token.txt"
python3 -B "$A/stub_vehicle.py" 1.0 > "$A/stub_vehicle.log" 2>&1 &
V=$!
ros2 run arena_autonomy race_ops --ros-args -p vehicle_id:=stubcar -p port:=18770 -p host:=0.0.0.0 -p "token_path:=$PWD/$A/stub_token.txt" \
  -p "log_path:=$PWD/$A/stub_ops_log.jsonl" -p client_timeout_s:=6.0 > "$A/stub_ops.log" 2>&1 &
O=$!
sleep "${1:-40}"
kill -INT $O $V; sleep 1; kill -9 $O $V 2>/dev/null
# pkill 금지: a3가 이 줄 때문에 다른 실행의 어댑터를 종료시켰다.
echo done

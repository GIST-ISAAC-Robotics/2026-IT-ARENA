#!/usr/bin/env bash
# 실행 전 helper 대조: 정지 오목 r1(양성, 원 함수와 같은 3.5초 창)과 r15 움직이는 원상자(음성).
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/medium_recessed_pass_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
POS=artifacts/validation/2026-11-03/irregular_actor_v1/run1
NEG=artifacts/validation/2026-11-08/medium_pass_connected_v1/straight_moving_lead/run1
mkdir -p "$A/controls"
bash "$ENV" /usr/bin/python3 -B "$A/outline_moving.py" "$POS" --start-offset 3.5 --output "$A/controls/positive_stationary_r1_offset35.json"; p1=$?
bash "$ENV" /usr/bin/python3 -B "$A/outline_moving.py" "$POS" --output "$A/controls/positive_stationary_r1_fullwindow.json"; p2=$?
bash "$ENV" /usr/bin/python3 -B "$A/outline_moving.py" "$NEG" --output "$A/controls/negative_r15_box_moving.json"; n1=$?
bash "$ENV" /usr/bin/python3 -B scripts/audit_actor_outline_returns.py "$NEG" --output "$A/controls/negative_r15_original_function.json"; n2=$?
echo "POS_OFFSET35=$p1 POS_FULL=$p2 NEG_MOVING=$n1 NEG_ORIGINAL=$n2" > "$A/controls/controls.exit"

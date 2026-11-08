#!/usr/bin/env bash
# 새 follow-profile 묶음 생성 → 검사·격리 추출·저장소/추출본 1회 재생 비교. ROS·Gazebo·Jetson 미사용.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-11-08/jetson_bundle_refresh_v1
python3 -B scripts/prepare_jetson_bundle.py --follow-profile-only   --follow-trace artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/run1/follow_decisions.jsonl   --follow-trace artifacts/validation/2026-11-08/current_corner_support_v1/run1/follow_decisions.jsonl   --archive build/jetson_follow_20261108_v1.tar.gz --manifest $A/bundle_manifest.json > $A/build.log 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > $A/build.exit
[ "$code" -eq 0 ] || exit "$code"
python3 -B $A/bundle_parity.py > $A/parity.log 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > $A/parity.exit
exit "$code"

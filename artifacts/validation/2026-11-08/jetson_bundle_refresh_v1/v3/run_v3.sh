#!/usr/bin/env bash
# r42 v3(v2 픽스처 footer 수정): 파생 픽스처 → v3 묶음 생성 → 검사·runtime 해시 대조·격리 추출·문서상 CLI 실행. ROS·Gazebo·Jetson 미사용.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
V=artifacts/validation/2026-11-08/jetson_bundle_refresh_v1/v3
step() { "$@"; code=$?; echo "ACTUAL_EXIT_CODE=$code" > "$V/$label.exit"; [ "$code" -eq 0 ] || exit "$code"; }
label=fixtures; step bash -c "python3 -B $V/make_fixtures.py > $V/fixtures.log 2>&1"
label=build; step bash -c "python3 -B scripts/prepare_jetson_bundle.py --follow-profile-only   --follow-trace artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/run1/follow_decisions.jsonl   --follow-trace artifacts/validation/2026-11-08/current_corner_support_v1/run1/follow_decisions.jsonl   --follow-trace artifacts/validation/2026-11-08/jetson_bundle_refresh_v1/v3/fixtures/r25_solo_first100.jsonl --follow-trace artifacts/validation/2026-11-08/jetson_bundle_refresh_v1/v3/fixtures/r41_handoff_first100.jsonl   --archive build/jetson_follow_20261108_v3.tar.gz --manifest $V/bundle_manifest.json > $V/build.log 2>&1"
label=cli_check; step bash -c "python3 -B $V/cli_check.py > $V/cli_check.log 2>&1"

#!/usr/bin/env bash
# r48 follow-profile 묶음 생성 → 검사 → 격리 추출 → 저장소/추출본 문서상 CLI 전체 trace 1회씩 → 비교. ROS·Gazebo·Jetson·네트워크 미사용.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
A=artifacts/validation/2026-11-09/jetson_r48_bundle_v1
X=build/jetson_follow_20261109_r48
T48=artifacts/validation/2026-11-08/observed_corner_continuity_r48_v1/run1/follow_decisions.jsonl
T44=artifacts/validation/2026-11-08/faster_entry_consistency_r44_v1/straight_moving_lead/run1/follow_decisions.jsonl
REPO=$(pwd)
CLEAN=(env -u PYTHONPATH -u PYTHONHOME -u PYTHONSTARTUP -u PYTHONUSERBASE PYTHONNOUSERSITE=1)
for v in $(compgen -e | grep -E '^(PYTHON|ROS|AMENT|COLCON|RMW|CYCLONEDDS|FASTRTPS)'); do CLEAN+=(-u "$v"); done
step() { label=$1; shift; "$@" > "$A/$label.log" 2>&1; code=$?; echo "ACTUAL_EXIT_CODE=$code" > "$A/$label.exit"; [ "$code" -eq 0 ] || exit "$code"; }

step hashes_before python3 -I -B $A/hashes.py before
step build python3 -B scripts/prepare_jetson_bundle.py --follow-profile-only --follow-trace $T48 --follow-trace $T44 \
  --archive build/jetson_follow_20261109_r48.tar.gz --manifest $A/bundle_manifest.json
step manifest_copy cp -n $A/bundle_manifest.json build/jetson_follow_20261109_r48.manifest.json
step verify python3 -B scripts/verify_follow_profile_bundle.py build/jetson_follow_20261109_r48.tar.gz $A/bundle_manifest.json --output $A/verify_result.json
step extract python3 -I -B $A/extract.py
for side in repo bundle; do
  if [ $side = repo ]; then R=$REPO; else R=$REPO/$X; fi
  for name in r48_corner r44_straight; do
    if [ $name = r48_corner ]; then TR=$T48; else TR=$T44; fi
    # cwd는 저장소 루트로 두어 -I 격리가 현재 디렉터리 import를 막는지 함께 본다.
    "${CLEAN[@]}" python3 -I -B $A/cli_run.py "$R" "$R/$TR" "$R/$A/cli_$side/$name" "$A/commands_${side}_$name.json.gz" "$REPO" \
      > "$A/cli_${side}_$name.log" 2>&1
    code=$?; echo "ACTUAL_EXIT_CODE=$code" > "$A/cli_${side}_$name.exit"
    tail -n 1 "$A/cli_${side}_$name.log" > "$A/run_${side}_$name.json"
  done
done
step hashes_after python3 -I -B $A/hashes.py after
step compare python3 -I -B $A/compare.py

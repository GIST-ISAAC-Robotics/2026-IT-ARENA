#!/bin/bash
# 후보 A의 스크래치 회귀와 저장 입력 재생. 사본만 수정하며 Gazebo/빌드를 실행하지 않는다.
set -eu
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-01/qualifying_corner_diagnosis_v1"
A="$HOME/arena_scratch/qc_diag"
python3 -I "$D/make_candidate_a.py" "$A/snap_combined" "$A/snap_candA" "$D/candidate_a.patch"
export ARENA_CANDIDATE_TREE="$A/snap_candA" PYTHONDONTWRITEBYTECODE=1
cd "$D/scratch_regression"
status=0
nice -n 19 python3 -m pytest -q -p no:cacheprovider test_candidate_a.py > pytest.log 2>&1 || status=$?
echo "pytest_exit=$status"
tail -25 pytest.log
if [ "${1:-}" = "all" ]; then
  mkdir -p "$D/replay"
  for run in combined_failed rise_only turn_only; do
    for mode in off on; do
      nice -n 19 python3 -I "$D/replay_candidate_a.py" "$run" "$A/snap_candA" "$mode" "$D/replay/candidate_a_${run}_$mode.json"
    done
  done
  nice -n 19 python3 -I "$D/arc_audit.py" "$A/snap_candA" "$D/arc_audit.json"
fi
exit $status

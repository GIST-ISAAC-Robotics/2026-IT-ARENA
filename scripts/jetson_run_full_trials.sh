#!/usr/bin/env bash
# 동일 설정의 1배속 두 번. 실패 판정도 결과로 보존하고 두 실행을 모두 확인한다.
set -eo pipefail
cd /home/q/it_arena_a3_20260928
source /opt/ros/humble/setup.bash
source install/setup.bash
recording=artifacts/validation/2026-09-24/record_replay/capture_v2/sensor_recording
for name in full_initial_a full_initial_b; do
    result=0
    python3 scripts/run_jetson_replay_trial.py "$recording" --output "artifacts/jetson_a3/$name" --rate 1 || result=$?
    echo "TRIAL_EXIT $name $result"
    test -f "artifacts/jetson_a3/$name/replay/report.json"
    python3 scripts/audit_replay_delivery.py "$recording" "artifacts/jetson_a3/$name/replay"
done
python3 scripts/audit_replay_delivery.py "$recording" artifacts/jetson_a3/half_initial/replay
python3 scripts/summarize_jetson_trials.py artifacts/jetson_a3/half_initial \
  artifacts/jetson_a3/full_initial_a artifacts/jetson_a3/full_initial_b \
  --output artifacts/jetson_a3/comparison_initial.json
test ! -e results_initial.tar.gz
tar -czf results_initial.tar.gz artifacts/jetson_a3
sha256sum results_initial.tar.gz
echo 'JETSON_INITIAL_TRIALS_COLLECTED'

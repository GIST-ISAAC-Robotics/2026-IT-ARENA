#!/usr/bin/env bash
# run1 후속 감사: 기존 run1 원본/평가를 덮어쓰지 않고 v2 기준(사후 보완)으로 새 파일에 계산한다.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/g2_entry_left_v1
run1=artifacts/validation/2026-10-29/g2_entry_gazebo_v1
sha256sum "$run1/run1_g2_evaluation.json" "$run1/run1/report.json" "$run1/run1_sources.zip" > "$base/run1_audit_before.sha256"
/usr/bin/python3 -B -m unittest discover -v -s "$base" -p 'test_g2_eval_core.py' > "$base/core_tests.log" 2>&1; echo $? > "$base/core_tests.exit"
/usr/bin/python3 -B "$base/g2_eval_v2.py" "$PWD/$run1/run1" --config "$run1/g2_config_v1.json" --rules-from "$base/g2_config_v2_left.json" \
  --result "$base/run1_followup_audit_v2.json" --plot "$base/run1_followup_path_v2.png" \
  --label 'POST-HOC audit of run1 with g2_acceptance_v2 rules; run1 registered verdict remains run1_g2_evaluation.json (v1 rules)' \
  > "$base/run1_audit.log" 2>&1; echo $? > "$base/run1_audit.exit"
sha256sum -c "$base/run1_audit_before.sha256" > "$base/run1_audit_after_check.log" 2>&1; echo $? >> "$base/run1_audit_after_check.log"

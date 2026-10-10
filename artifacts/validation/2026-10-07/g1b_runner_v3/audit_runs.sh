#!/usr/bin/env bash
# 실행 뒤 사후 상태/자세 감사 + 독립 출처 감사(읽기 전용, 원본 report 불변). 인자: 실행 폴더 이름들.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
freeze=${FREEZE:-artifacts/validation/2026-10-07/g1b_unit_v6/source_freeze.json}
audit_dir=${AUDIT_DIR:-artifacts/validation/2026-10-07/g1b_source_audit_v3}
mkdir -p "$audit_dir"
for name in "$@"; do
  run=artifacts/validation/2026-10-07/$name
  post=$run/postcoverage_v2.json
  [ -e "$post" ] || python3 -B scripts/audit_g1_traffic_coverage.py $run --output $post > /dev/null
  python3 -c "import json;p=json.load(open('$post'));print('POST $name', p['passed'], p['applicability'], 'raw', p['raw_status_samples'], 'age', round(p['trace_status_max_age_s'],3), 'hold', p['held_pose_segments'], round(p['max_held_pose_speed_mps'],6), 'stop', p['stop_last_half_second_pose_segments'], round(p['stop_max_pose_speed_mps'],6))"
  out=$audit_dir/$name.json
  [ -e "$out" ] || python3 -B scripts/audit_g1_traffic_sources.py $run --freeze $freeze --postcoverage $post --output $out > /dev/null
  python3 -c "import json;a=json.load(open('$out'));print('SRC $name', a['passed'], 'runtime', a['runtime_source_files'], 'freeze', a['source_freeze_files'], [k for k,v in a['checks'].items() if not v])"
done

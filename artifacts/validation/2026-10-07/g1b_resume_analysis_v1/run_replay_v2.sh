#!/usr/bin/env bash
# 재개 등속 seed 0(수정 소스) 실행의 8.72초 일시 경계 상실 재생(분석 전용).
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-10-07/g1b_resume_analysis_v1
R=artifacts/validation/2026-10-07/g1b_resume_v1_slow_s0
python3 -B scripts/replay_g1b_follow.py $R --first 3.5 --last 16.4 --output $A/replay_resume_slow_zero.json
python3 -B - <<'PY'
import json
r=json.load(open('artifacts/validation/2026-10-07/g1b_resume_analysis_v1/replay_resume_slow_zero.json'))
for row in r['rows']:
    if 8.3 <= row['t'] <= 9.0 and (row.get('boundary_selection') or row['reason'] != 'local_follow'):
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in row.items() if k in (
            't','scan_s','reason','boundary_selection','boundary_candidate_ahead_m','fresh_boundary_accepted','boundary_from_memory',
            'boundary_age_s','boundary_protrusions_removed','left_unfiltered_ahead_m','right_unfiltered_ahead_m','measured_speed_mps')})
PY

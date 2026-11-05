#!/usr/bin/env bash
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05/right_angle_course_v1"
WORLD=course_world bash "$D/post.sh" run4
T=$(python3 - "$D/run4/trajectory.jsonl" <<'PY'
import json,sys
rows=[json.loads(l) for l in open(sys.argv[1],encoding='utf-8')]
hold=[r['time'] for r in rows if r['autonomy']['state']=='FOLLOW_BOUNDARY_UNCONFIRMED']
first=hold[0] if hold else 13.0
# 판단 기록의 스캔 주기(0.1 s, x.x2)에 맞춘 관심 시각
base=round(first-0.3,1)
print(' '.join(f'{base+0.1*i+0.02:.2f}' for i in range(8))+' 16.02 19.02')
PY
)
WS_DIR="$HOME/arena_scratch/right_angle_v1/ws2" bash "$D/diag.sh" run4 trial_patch/run4_diag $T

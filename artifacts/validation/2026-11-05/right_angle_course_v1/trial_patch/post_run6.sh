#!/usr/bin/env bash
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05/right_angle_course_v1"
WORLD=iso_world bash "$D/post.sh" run6
T=$(python3 - "$D/run6/trajectory.jsonl" <<'PY'
import json,sys
rows=[json.loads(l) for l in open(sys.argv[1],encoding='utf-8')]
hold=[r['time'] for r in rows if r['autonomy']['state']=='FOLLOW_BOUNDARY_UNCONFIRMED' and r['progress_m']>9]
f=hold[0]
print(' '.join(f'{int(f*10)/10-0.3+0.1*i+0.02:.2f}' for i in range(6))+f' {int(f*10)/10+3.02:.2f}')
PY
)
echo "diag_times=$T"
WS_DIR="$HOME/arena_scratch/right_angle_v1/ws2" bash "$D/diag.sh" run6 trial_patch/run6_diag $T

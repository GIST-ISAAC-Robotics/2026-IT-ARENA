#!/usr/bin/env bash
# 공식 팀 월드 단독 기록(11/5 OFF 기준·r15 ON)을 ws(현행)·ws2(v3 후보) 코드로 재생해 판단 변경 수를 센다(평가 전용).
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05"
for W in ws ws2; do
  cd "$HOME/arena_scratch/right_angle_v1/$W" || exit 2
  set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
  export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
  for R in solo_jr60p20_baseline_v1/run1 solo_straight_closed_loop_v1/run1; do
    python3 -B - "$D/$R/follow_decisions.jsonl" "$W" "$R" <<'PY'
import json, sys
from arena_autonomy.follow_trace import replay_events
with open(sys.argv[1], encoding='utf-8') as s:
    r = replay_events(json.loads(l) for l in s if l.strip())
m = r.get('mismatches') or []
print(json.dumps(dict(code=sys.argv[2], record=sys.argv[3], calls=r['calls'], matched=r['matched'],
                      first_mismatch_seq=m[0]['seq'] if m else None)))
PY
  done
done

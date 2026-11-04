"""재개 뒤 정답 궤적과 표시 상태 변화를 시간순으로 본다. 인자: 실행 폴더, 시작 단계 이름, 순번."""
import json
import sys
from pathlib import Path

run = Path(sys.argv[1])
step_name = sys.argv[2] if len(sys.argv) > 2 else 'start_response_dropped'
which = int(sys.argv[3]) if len(sys.argv) > 3 else 0
report = json.loads((run/'report.json').read_text(encoding='utf-8'))
steps = report['steps']
for row in steps:
    if row['step'] in ('passed_id30_naturally', 'placed', 'resume_closed', 'stopped', step_name):
        print(row['step'], row.get('sim_s'), row.get('side'), (row.get('truth') or [None]*3)[:3], row.get('s'), row.get('route'), row.get('marker_side'))
start = [row for row in steps if row['step'] == step_name][which]
t0, wall0 = start['truth'][0], start['wall_s']
poses = [json.loads(line)[:3] for line in (run/'ego_poses.jsonl').read_text().splitlines()]
poses = [row for row in poses if row[0] >= t0]
print('poses', len(poses), 'last', poses[-1] if poses else None)
next_t = t0
for row in poses:
    if row[0] >= next_t:
        print(round(row[0], 2), round(row[1], 3), round(row[2], 3))
        next_t += 1.
history = json.loads((run/'ops_history.json').read_text(encoding='utf-8'))
last = None
for row in history:
    if row['wall'] < wall0:
        continue
    d = row['driving']
    key = (row['state'], d['state'], d['reason'], d['behavior'], d['side'], tuple(d['marker_ids'] or []), d['safety_reason'])
    if key != last:
        print(round(d['sim_s'] or 0, 2), key, row['speeds']['plan_mps'], row['speeds']['protected_mps'])
        last = key

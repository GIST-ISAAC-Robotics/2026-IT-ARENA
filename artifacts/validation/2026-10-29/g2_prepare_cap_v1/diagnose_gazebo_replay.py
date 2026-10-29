"""Gazebo schema5 기록의 재생 불일치가 어느 코드 변경에서 생겼는지 가린다. 새 주행 아님.

인수: 소스 루트(src/를 포함하는 디렉터리), 출력 JSON 이름. 원본 기록은 읽기만 한다.
"""
import hashlib
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
SOURCE = Path(sys.argv[1]).resolve()
sys.path[:0] = [str(SOURCE/'src/arena_autonomy'), str(SOURCE/'src/arena_vehicle_interface')]
import arena_autonomy.follow_trace as ft
from arena_autonomy.follow_trace import replay_events

assert Path(ft.__file__).resolve().is_relative_to(SOURCE), ft.__file__
results = []
for run in ('run1', 'run2'):
    path = ROOT/'artifacts/validation/2026-10-29/g2_handoff_gazebo_v1'/run/'g2_decisions.jsonl'
    events = [json.loads(line) for line in path.open(encoding='utf-8')]
    by_seq = {e['seq']: e for e in events}
    replay = replay_events(events)
    first = []
    for m in replay['mismatches'][:3]:
        data = by_seq[m['seq']]['data']['input']
        first.append(dict(seq=m['seq'], reference_s=data['reference_s'], measured_speed_mps=data['speed_mps'],
            expected=[m['expected'][0], m['expected'][1], m['expected'][2].get('reason'), m['expected'][2].get('handoff_state')],
            actual=[m['actual'][0], m['actual'][1], m['actual'][2].get('reason'), m['actual'][2].get('handoff_state')]))
    negative = [e['seq'] for e in events if e['kind'] == 'call' and e['data']['input']['speed_mps'] < 0]
    results.append(dict(trace=path.relative_to(ROOT).as_posix(), calls=replay['calls'], matched=replay['matched'],
        first_mismatches=first, negative_speed_call_seq=negative[:10], negative_speed_calls=len(negative)))
    print(json.dumps(results[-1]), flush=True)
out = dict(source_root=str(SOURCE), follow_trace_module=str(Path(ft.__file__).resolve()),
    follow_trace_sha256=hashlib.sha256(Path(ft.__file__).read_bytes()).hexdigest(), results=results,
    scope='Diagnostic replay of preserved Gazebo schema5 traces; not new driving.')
(BASE/sys.argv[2]).write_text(json.dumps(out, indent=2)+'\n', encoding='utf-8')

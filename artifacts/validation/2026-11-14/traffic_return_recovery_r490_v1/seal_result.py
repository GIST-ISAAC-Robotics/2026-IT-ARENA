"""원 실행 자료의 최초 두 정지 입력만 발췌한다. 제어 재생/판정 변경 없음."""
import collections
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
R = A / 'run1'
D = A / 'failure_inputs'
D.mkdir(exist_ok=True)

def read(p):
    return json.loads(p.read_bytes())

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def write(name, value):
    (D / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

raw = (R / 'follow_decisions.jsonl').read_bytes().splitlines(keepends=True)
pending = {}
selected = []
index = []
reasons = collections.Counter()
first = []
for number, line in enumerate(raw, 1):
    row = json.loads(line)
    kind, v = row['kind'], row['data']
    t = None
    if kind == 'runtime_plan_submitted':
        t = v['input']['reference_s']
        pending[(v['generation'], v['sequence'])] = (t, number)
    elif kind == 'runtime_plan_completed':
        t, input_line = pending[(v['generation'], v['sequence'])]
        output = v['output']
        if t >= 12.32 and output[0] == 0:
            reasons[output[2].get('reason')] += 1
        if v['sequence'] in (74, 87):
            entry = output[2]['moving_pass']['entry']
            candidates = [{k: x for k, x in c.items() if k != 'path_points'} for c in entry['candidates']]
            first.append(dict(generation=v['generation'], sequence=v['sequence'], reference_s=t,
                input_line=input_line, completion_line=number, output_speed_mps=output[0],
                output_steering_rad=output[1], reason=output[2]['reason'],
                entry_reason=entry['reason'], candidates=candidates))
    elif kind == 'call':
        t = v['input']['reference_s']
    elif kind == 'route_publication':
        t = v.get('reference_s')
    if kind in ('header', 'runtime_evaluator_header', 'runtime_factory_policy') or (
        t is not None and (10.91 <= t <= 11.13 or 12.21 <= t <= 12.53)
    ):
        selected.append(line)
        index.append(dict(source_line=number, seq=row['seq'], kind=kind, reference_s=t))
(D / 'follow_records.jsonl').write_bytes(b''.join(selected))
write('record_index.json', index)

rows = [json.loads(line) for line in (R / 'trajectory.jsonl').read_bytes().splitlines()]
transitions = []
last = None
for row in rows:
    if row.get('elapsed_since_green_s') is None:
        continue
    state = (row.get('drive_speed_mps', 0) > .001, row.get('safe_speed_mps', 0) > .001)
    if state != last:
        transitions.append({k: row.get(k) for k in ('t', 'progress_m', 'ground_speed_mps',
            'drive_speed_mps', 'safe_speed_mps', 'safety_reason', 'wall_collision')})
        last = state
write('first_stops.json', dict(actual_run_count=1, new_replay=False, first_stops=first,
    trajectory_command_transitions=transitions, zero_plan_reasons_after_12_32=dict(reasons),
    limitation='기록된 후보 점유 거절의 근거이며 어느 물체/벽 반환인지는 별도 분석 전 미확정. 정답 위치는 제어 입력으로 사용하지 않음.'))
write('README.json', dict(scope='원 r490 최초 짧은 정지와 지속 정지의 입력/출력 발췌. 새 계산/주행 아님.',
    trace=dict(path='run1/follow_decisions.jsonl', sha256=sha(R / 'follow_decisions.jsonl')),
    evidence={str(p.relative_to(A)): sha(p) for p in (R / 'report.json', R / 'trajectory.jsonl',
        A / 'prepared.json', A / 'sources.zip', A / 'integrity.json',
        A / 'execution_v1/owned_processes_after.json')},
    extraction='follow_records.jsonl은 해당 원본 행의 byte 그대로이며 record_index.json의 source_line은 1-based.',
    truth_boundary='trajectory progress/contact는 평가용. 제어의 scan/history/epoch/기준시각은 원 follow 행에 보존.'))
print(json.dumps(dict(records=len(selected), first_stops=[dict(sequence=x['sequence'], reference_s=x['reference_s'],
    candidate_reasons=[c['reason'] for c in x['candidates']]) for x in first],
    zero_plan_reasons_after_12_32=dict(reasons))))

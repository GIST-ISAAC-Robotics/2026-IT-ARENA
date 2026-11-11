"""r84 저장 판단의 감속 후보 채택과 마지막 정체 원인만 요약한다."""
import collections
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
trace = A / 'two_actor_run1/run1/follow_decisions.jsonl'
digest = hashlib.sha256(trace.read_bytes()).hexdigest()
counts = collections.Counter()
last_counts = collections.Counter()
changes = []
last_signature = None
samples = []
for line in trace.open(encoding='utf-8'):
    event = json.loads(line)
    if event['kind'] != 'call':
        continue
    data = event['data']; inp = data['input']; out = data['output']
    t = inp['reference_s']; normal = out[2].get('normal', {})
    partial = normal.get('partial_observation', {})
    recheck = normal.get('selector_guard_speed_recheck', {})
    counts['calls'] += 1
    counts['recheck_applied'] += bool(recheck.get('applied'))
    counts['recheck_rejected'] += bool(recheck) and not recheck.get('applied', False)
    counts['positive_commands'] += out[0] > 0
    if t >= 59.9:
        signature = (normal.get('reason'), normal.get('selector_reason'),
                     partial.get('reason'), partial.get('latch_reason'))
        last_counts[str(signature)] += 1
        if signature != last_signature:
            changes.append(dict(seq=event['seq'], time_s=t, speed_mps=inp['speed_mps'],
                command=out[:2], reasons=signature, partial=partial))
            last_signature = signature
        if not samples or t >= samples[-1]['time_s'] + .5:
            samples.append(dict(seq=event['seq'], time_s=t, speed_mps=inp['speed_mps'],
                command=out[:2], normal=normal))
result = dict(scope='r84 recorded decisions only, no new run or truth control',
    trace_sha256=digest, counts=dict(counts), last_window_counts=dict(last_counts),
    changes=changes, samples=samples,
    trace_unchanged=hashlib.sha256(trace.read_bytes()).hexdigest() == digest)
for item in changes:
    evidence = item['partial'].get('latch_evidence')
    if evidence:
        item['partial']['latch_evidence'] = {k:v for k,v in evidence.items() if k != 'correspondence_context'}
for item in samples:
    partial = item['normal'].get('partial_observation', {})
    evidence = partial.get('latch_evidence')
    if evidence:
        partial['latch_evidence'] = {k:v for k,v in evidence.items() if k != 'correspondence_context'}
    item['normal'] = {k:v for k,v in item['normal'].items() if k in
        ('reason','selector_reason','partial_observation','boundary_candidate_reason','fresh_boundary_accepted')}
result['evidence_fixture'] = 'latch_sensor_fixture.json'
(A / 'decision_audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps(dict(counts=result['counts'], last_counts=result['last_window_counts'],
    changes=[dict(time_s=v['time_s'], speed=v['speed_mps'], reasons=v['reasons'],
                  latch_time=v['partial'].get('latch_time_s'),
                  evidence_pairs=(v['partial'].get('latch_evidence') or {}).get('pairs'))
             for v in changes[:12]], trace_unchanged=result['trace_unchanged']), ensure_ascii=False, indent=2))

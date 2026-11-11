"""r98 취득/판단 시각과 Jetson 단계 비용의 저장 대조. 새 주행/지연 주입 아님."""
from collections import Counter
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
COST = HERE.parent/'control_10hz_cost_r98_v1'
manifest_path = COST/'bundle_manifest.json'
manifest = json.loads(manifest_path.read_bytes())
trace = ROOT/manifest['trace']
jetson = COST/'jetson_stages.json'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
before = {str(p.relative_to(ROOT)): sha(p) for p in (trace, manifest_path, jetson)}
profile = json.loads(jetson.read_bytes())
costs = {r['seq']: sum(s['wall_ms'] for s in r['stages'].values())/1000.
         for r in profile['per_call_stages']}
rows = []
with trace.open(encoding='utf8') as stream:
    for line in stream:
        e = json.loads(line)
        if e['kind'] == 'header':
            header = e['data']
        if e['kind'] != 'call':
            continue
        data = e['data']; x = data['input']; scan = x['scan']
        assert scan['representation'] == 'native_measurements_v1'
        first = scan['measurements']['acquired_at_s'][0]
        last = scan['measurements']['acquired_at_s'][-1]
        cost = costs[e['seq']]
        detail = data['output'][2]; normal = detail.get('normal') or detail
        rows.append(dict(seq=e['seq'], reference_s=x['reference_s'],
            first_age_s=x['reference_s']-first, last_age_s=x['reference_s']-last,
            stage_sum_s=cost, estimated_apply_first_age_s=x['reference_s']-first+cost,
            speed_mps=x['speed_mps'], command_mps=data['output'][0],
            constant_speed_distance_m=abs(x['speed_mps'])*cost,
            mode=detail.get('handoff_state'), reason=normal.get('selector_reason') or normal.get('reason')))
assert len(rows) == 610 and len(costs) == len(rows)
timeout = min(.3, header['motion_config']['scan_timeout_s'])
def distribution(values):
    values=sorted(values)
    def p(q):
        at=(len(values)-1)*q; lo=int(at); hi=min(lo+1,len(values)-1)
        return values[lo]*(hi-at)+values[hi]*(at-lo) if hi != lo else values[lo]
    return dict(count=len(values),p50=p(.5),p95=p(.95),maximum=values[-1])
def counts(selected):
    return dict(count=len(selected),reasons=dict(Counter(str(r['reason']) for r in selected)))
report=dict(
    scope='fixed trajectory timing-envelope analysis; no async commands applied; stage sums omit orchestration overhead',
    calls=len(rows), scan_timeout_s=timeout, cost=distribution([r['stage_sum_s'] for r in rows]),
    input_first_age=distribution([r['first_age_s'] for r in rows]),
    estimated_apply_first_age=distribution([r['estimated_apply_first_age_s'] for r in rows]),
    constant_speed_distance=distribution([r['constant_speed_distance_m'] for r in rows]),
    apply_outside_existing_scan_age=counts([r for r in rows if r['estimated_apply_first_age_s'] >= timeout]),
    cost_over_200ms=counts([r for r in rows if r['stage_sum_s'] > .2]),
    under_200ms_but_scan_stale=counts([r for r in rows if r['stage_sum_s'] <= .2 and r['estimated_apply_first_age_s'] >= timeout]),
    tail=sorted(rows,key=lambda r:r['stage_sum_s'],reverse=True)[:12],
    source_sha256=before, input_unchanged=all(sha(ROOT/p)==h for p,h in before.items()))
target=HERE/'report.json'
with target.open('x',encoding='utf8') as stream:json.dump(report,stream,ensure_ascii=False,indent=2);stream.write('\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('tail','source_sha256')},ensure_ascii=False))

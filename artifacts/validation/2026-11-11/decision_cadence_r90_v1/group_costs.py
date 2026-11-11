"""이미 회수한 비용과 입력 원천 시각만 대조한다. 제어기를 재실행하지 않는다."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import statistics

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
TRACE = ROOT/'artifacts/validation/2026-11-11/native_traffic_baseline_r87_v1/two_actor_run1/run1/follow_decisions.jsonl'
COST = A.parent/'occluder_cpu_r89_v1/jetson_stages.json'
cost = json.loads(COST.read_bytes())
cost_by_seq = {r['seq']:sum(r['stages'][k]['wall_ms'] for k in ('motion_scene','follow_command'))
               for r in cost['per_call_stages']}
selected = {json.loads(r)['seq'] for r in (A/'busy_recorded/admitted.jsonl').read_text().splitlines()}
groups = defaultdict(list)
all_sources, selected_sources = set(), set()
last_source = None
with TRACE.open(encoding='utf8') as f:
    for line in f:
        e = json.loads(line)
        if e['kind'] == 'reset':
            last_source = None
        if e['kind'] != 'call':
            continue
        d = e['data']['input']
        scan = d['scan']['measurements']
        key = (d['epoch'], scan['source_frame_id'], scan['acquired_at_s'][0])
        groups['new_source' if last_source != key else 'same_source'].append(cost_by_seq[e['seq']])
        all_sources.add(key)
        if e['seq'] in selected:
            selected_sources.add(key)
        last_source = key

def summary(v):
    ordered = sorted(v)
    # 선형 보간 percentile, numpy 없이 기록값만 계산한다.
    def pct(p):
        loc = (len(v)-1)*p
        lo = int(loc)
        return ordered[lo]+(ordered[min(lo+1,len(v)-1)]-ordered[lo])*(loc-lo)
    return dict(count=len(v), mean=statistics.mean(v), p50=pct(.5), p95=pct(.95),
                maximum=max(v), over_100ms=sum(x>100 for x in v))

r = dict(scope='same saved trace grouped by source identity; no controller execution',
    cost_scope='motion_scene plus follow_command wall time, offline restore excluded',
    trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
    cost_sha256=hashlib.sha256(COST.read_bytes()).hexdigest(),
    groups_ms={k:summary(v) for k,v in groups.items()},
    distinct_sources=len(all_sources), busy_selected_sources=len(selected_sources),
    busy_missed_sources=len(all_sources-selected_sources))
assert r['trace_sha256'] == cost['trace_sha256']
(A/'cost_by_source.json').write_text(json.dumps(r,indent=2)+'\n', encoding='utf8')
print(json.dumps(r))

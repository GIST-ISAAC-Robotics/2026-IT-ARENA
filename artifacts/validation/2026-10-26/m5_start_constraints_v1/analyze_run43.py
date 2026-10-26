"""저장된 새 궤적에서 정지 원인과 명령 검사의 근거만 추출한다."""
import hashlib
import json
from pathlib import Path

base = Path(__file__).resolve().parent
results = {}
for path in sorted((base/'run43_deferred_center_motion/data').glob('grid*/report.json')):
    raw = path.read_bytes(); report = json.loads(raw)
    first = report['summary']['first_motion_s']
    intervals = []
    for c in report['calls']:
        if c['t_s'] < first or c['explicit_stop']: continue
        m = c['metadata']; lat = m.get('m3_lateral', {})
        key = (m['reason'], m.get('m3_candidate'), m.get('swept_clearance_m'))
        if intervals and tuple(intervals[-1]['key']) == key:
            intervals[-1]['last_s'] = c['t_s']; intervals[-1]['calls'] += 1
        else:
            intervals.append(dict(key=key, first_s=c['t_s'], last_s=c['t_s'], calls=1,
                request_mps=c['request_mps'], measured_mps=m.get('measured_speed_mps'),
                candidate_reasons={n:v.get('reasons') for n,v in lat.get('checks',{}).items()},
                command_observation=m.get('m3_envelope',{}).get('observation'),
                release=lat.get('release_gate',{}).get('reason')))
    results[path.parent.name] = dict(input_sha256=hashlib.sha256(raw).hexdigest(),
        unchanged=path.read_bytes()==raw, intervals=intervals,
        stop_intervals=[r for r in intervals if r['request_mps'] == 0])
(base/'run43_stop_analysis.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
for name,r in results.items():
    print(name)
    for interval in r['stop_intervals'][:12]:
        print(json.dumps(interval,ensure_ascii=False))

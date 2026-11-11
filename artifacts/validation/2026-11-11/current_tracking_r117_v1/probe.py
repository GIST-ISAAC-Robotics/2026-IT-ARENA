"""r114 실제 도착 이전의 경로만 현재 입력에 대조한다. 원 궤적 고정 진단."""
import collections
import hashlib
import json
from pathlib import Path
import statistics
import time

from candidate import proposal, current_candidate
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene

A = Path(__file__).resolve().parent
SOURCE = A.parent / 'fresh_submission_r114_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    rows = [json.loads(s) for s in SOURCE.open(encoding='utf8')]
    follow, motion = make_controller(rows[0]['data'])
    calls = [r['data'] for r in rows if r['kind'] == 'call']
    events = []
    last_event_time = None
    for row in rows:
        d = row['data']
        if row['kind'] == 'call':
            last_event_time = d['input']['scan']['measurements']['acquired_at_s'][0] + d['delivery']['source_age_s']
            item = proposal(d['input'], d['output']) if d['delivery']['kind'] == 'candidate' else None
            events.append((last_event_time, item))
        elif row['kind'] == 'reset' and last_event_time is not None:
            events.append((last_event_time, None))
    events.sort(key=lambda p: p[0])
    costs, results = [], []
    index = 0
    plan = None
    for call in calls:
        data, scan, history = restore_call(call['input'], motion)
        now = data['reference_s']
        # 現 입력 이후의 계산/도착·평가 actor/지도는 조회하지 않는다.
        while index < len(events) and events[index][0] <= now:
            plan = events[index][1]
            index += 1
        start = time.perf_counter()
        scene = motion_scene(scan, history, now, timing_verified=data['timing_verified'],
            epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        out = current_candidate(plan, data, scan, history, scene, follow.config)
        costs.append((time.perf_counter()-start)*1000)
        out.pop('path_points', None)
        results.append(dict(reference_s=now, **out))
    ordered = sorted(costs)
    summary = dict(calls=len(results), reasons=dict(collections.Counter(r['reason'] for r in results)),
        eligible=sum(r['eligible'] for r in results),
        cost_ms=dict(p50=statistics.median(costs), p95=ordered[int(.95*len(ordered))], maximum=max(costs)),
        source_sha256=digest, source_unchanged=hashlib.sha256(SOURCE.read_bytes()).hexdigest()==digest,
        scope='Saved sensor trajectory only, no closed-loop command. Current projection/support included; data restoration excluded. Mode/road contract and output feedback consistency not complete.')
    with (A/'comparison.json').open('x') as f:
        json.dump(summary, f, indent=2)
    with (A/'details.jsonl').open('x') as f:
        for r in results:
            f.write(json.dumps(r)+'\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()

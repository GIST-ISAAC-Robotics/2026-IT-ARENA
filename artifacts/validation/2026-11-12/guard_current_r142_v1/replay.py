"""r139 양수 GUARD 뒤의 다음 저장 입력에서 현재 재검사 후보를 비교한다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
from candidate import rebase, supported, ROOT

A = Path(__file__).resolve().parent
SOURCE = ROOT/'artifacts/validation/2026-11-11/guard_recheck_r139_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    events = [json.loads(line) for line in SOURCE.open(encoding='utf8')]
    assert events[-1]['kind'] == 'footer'
    owner, motion = make_controller(events[0]['data'])
    calls = [e for e in events if e['kind']=='call']
    results = []; reasons = Counter(); costs = []; fixture = None
    for i, event in enumerate(calls[:-1]):
        saved = event['data']
        if not supported(saved['output']) or saved['delivery']['kind']!='candidate':
            continue
        later = calls[i+1]
        if any(e['kind']=='reset' for e in events[event['seq']+1:later['seq']]):
            reasons['reset_boundary_excluded'] += 1
            continue
        data, scan, history = restore_call(later['data']['input'], motion)
        # 실제 도착 순간 입력을 기록하지 않았으므로 다음 제출 입력을 쓴 반사실 비교다.
        arrived = saved['input']['scan']['measurements']['acquired_at_s'][0]+saved['delivery']['source_age_s']
        assert data['reference_s'] >= arrived-1e-9
        began = time.perf_counter()
        scene = motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                             epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
        result = rebase(saved['output'],saved['input'],data,scan,scene,history,owner.config)
        cost = 1000*(time.perf_counter()-began); costs.append(cost)
        assert result is not None
        assert result['speed_mps'] <= saved['output'][0]
        assert result['reference_s'] == data['reference_s']
        reasons[result['reason']] += 1
        if result['speed_mps'] > 0:
            assert 0 <= result['reference_s']-result['path_source_s'] < owner.config.memory_ttl_s
            assert result['region']['masked_tail_extent_m'] <= .01
            assert result['region']['masked_front_samples'] == 0
            assert result['independent_guard_required'] and result['runtime_authorized'] is False
            if fixture is None:
                fixture = dict(header=events[0]['data'], original=saved,
                               current=later['data']['input'], output=result)
        results.append(dict(seq=event['seq'],next_seq=later['seq'],arrived_s=arrived,
                            snapshot_after_arrival_s=data['reference_s']-arrived,
                            before=saved['output'][:2], current=result, computation_ms=cost))
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == digest
    costs.sort()
    output = dict(source_sha256=digest,source_unchanged=True,candidates=len(results),reasons=dict(reasons),
                  positive=sum(r['current']['speed_mps']>0 for r in results),
                  computation_ms=dict(p50=costs[len(costs)//2],p95=costs[int(.95*len(costs))],maximum=costs[-1]),
                  results=results,
                  scope='Next recorded submission input, not actual arrival input. No publication, state change or closed-loop driving result.')
    with (A/'comparison.json').open('x',encoding='utf8') as f:json.dump(output,f,indent=2)
    if fixture is not None:
        with (A/'fixture.json').open('x',encoding='utf8') as f:json.dump(fixture,f,indent=2)
    print(json.dumps({k:v for k,v in output.items() if k!='results'}))


if __name__ == '__main__':main()

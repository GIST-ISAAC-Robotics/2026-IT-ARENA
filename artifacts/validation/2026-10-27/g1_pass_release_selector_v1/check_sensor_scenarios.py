"""고정 운동의 순차 광선 장면 비교. 실제 명령으로 움직이는 폐루프 시험이 아니다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
for part in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts'):
    sys.path.insert(0, str(ROOT/part))
sys.path.insert(0, str(HERE))
import numpy as np
from arena_autonomy.local_follow import FollowConfig
from pass_release_selector import PassReleaseSelector
from synthetic_world import Body, World


def cases():
    yield 'walls_only', World(.35, 0.), 'SOLO'
    for side in (1., -1.):
        label = 'left' if side > 0 else 'right'
        passing = Body(.2, .15, lambda t, s=side: (-.60+t, s*.22), 'passing')
        yield label+'_forward_pass', World(.35, 0., [passing]), 'RELEASED'
        retained = Body(.2, .15, lambda t, s=side: (.35*t-.10, s*.22), 'retained')
        yield label+'_one_pass_one_remains', World(.35, 0., [passing, retained]), 'GUARD'
    backward = Body(.2, .15, lambda t: (.08-.05*t, .22), 'falls_back')
    yield 'visible_then_behind', World(.35, 0., [backward]), 'GUARD'
    retained = Body(.2, .15, lambda t: (.35*t-.10, .22), 'retained')
    yield 'alongside_stays', World(.35, 0., [retained]), 'GUARD'


def run(name, world, expected):
    cfg = FollowConfig(partial_observation_policy='bounded_local_v4',
        boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate')
    selector = PassReleaseSelector(cfg)
    rows = []
    for t in np.round(np.arange(.2, 3.01, .1), 6):
        scene = world.scene(float(t))
        # 후보 출력은 상태 선택을 격리하기 위해 고정한다. 경로 추종·제동 결과로 해석하지 않는다.
        baseline = (1., 0., {'reason':'local_follow', 'path_points':world.path().tolist()})
        parallel = (.2, 0., {'reason':'parallel_hold_follow', 'path_points':world.path().tolist()})
        start = time.perf_counter()
        result = selector.select(scene, baseline, parallel, .35, 0., .02)
        cost = (time.perf_counter()-start)*1000
        rows.append({'t':float(t), 'mode':selector.mode, 'speed':result[0], 'detail':result[2],
                     'cost_ms':cost, 'evaluation_only':{b.name:world.relative(b,float(t)) for b in world.bodies}})
    seen_guard = any(r['mode']=='GUARD' for r in rows)
    checks = {'expected_final_mode':rows[-1]['mode']==expected,
              'guard_seen_when_objects':seen_guard if world.bodies else not seen_guard,
              'no_false_release':all(r['mode']!='RELEASED' for r in rows) if expected=='GUARD' else True,
              'all_finite_output':all(np.isfinite(r['speed']) and 0 <= r['speed'] <= 1 for r in rows)}
    summary = {'expected_final_mode':expected, 'final_mode':rows[-1]['mode'],
        'modes':dict(Counter(r['mode'] for r in rows)), 'checks':checks,
        'pass':all(checks.values()), 'cost_ms_max':max(r['cost_ms'] for r in rows)}
    transitions, old = [], None
    for row in rows:
        if row['mode'] != old:
            transitions.append({k:row[k] for k in ('t','mode','evaluation_only')})
            old = row['mode']
    summary['transitions'] = transitions
    return rows, summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    paths = [HERE/n for n in ('pass_release_selector.py','synthetic_world.py','check_sensor_scenarios.py')]
    hashes = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    report = {'scope':'synthetic sequential lidar, fixed motion and supplied candidate commands; no closed loop',
              'sources':hashes, 'cases':{}}
    for name, world, expected in cases():
        rows, summary = run(name,world,expected)
        with (args.output/(name+'_rows.jsonl')).open('x',encoding='utf-8') as f:
            for row in rows:f.write(json.dumps(row,ensure_ascii=False)+'\n')
        report['cases'][name] = summary
        print(json.dumps({name:summary},ensure_ascii=False),flush=True)
    report['sources_unchanged'] = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    (args.output/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    sys.exit(0 if report['sources_unchanged'] and all(r['pass'] for r in report['cases'].values()) else 1)

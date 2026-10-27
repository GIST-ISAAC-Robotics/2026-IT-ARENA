"""run8 마지막 센서 스캔과 제어 호출 건너뜀 반례. 실제 과거 동적 입력 재생은 아님."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'),str(ROOT/'tests')]
from arena_autonomy.partial_observation import PartialObservationPolicy
from test_local_follow import history,scan


def main():
    path = OUT/'run8_lap/last_scan.json'
    raw = json.loads(path.read_text())
    raw['ranges'] = [float('nan') if value is None else value for value in raw['ranges']]
    h = history()
    sample = scan(.5,raw['ranges'])
    sample.angle_min = raw['angle_min']
    sample.angle_increment = raw['angle_increment']
    points,_,_ = h.project(sample.ranges,sample.angle_min,sample.angle_increment,
        sample.range_min,sample.range_max,.5,sample.time_increment,.6,'both',True)
    unknown = [i for i,r in enumerate(sample.ranges) if not np.isfinite(r) and abs(sample.angle_min+i*sample.angle_increment)<np.radians(100)]
    rows=[]
    p=PartialObservationPolicy()
    # 동일한 실제 마지막 점군을 기억한 가정. 앞선 주행 중 점군을 복원했다고 주장하지 않는다.
    p._remember(points,.5,.4,True)
    rows.append(dict(case='same_static_points',result=p.evaluate(sample,points,h,.6,1.,.3674,.3674)))
    p=PartialObservationPolicy()
    p._remember(points,.1,0.,True)
    rows.append(dict(case='observation_call_skipped_500ms',result=p.evaluate(sample,points,h,.6,1.,.3674,.3674)))
    later=scan(.6,raw['ranges']);later.angle_min=sample.angle_min;later.angle_increment=sample.angle_increment
    rows.append(dict(case='after_expired_history',result=p.evaluate(later,points,h,.7,1.,.3674,.3674)))
    result=dict(scope='stationary_saved_scan_and_skipped_call_counterfactual_not_full_run_replay',
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (path,Path(__file__),ROOT/'src/arena_autonomy/arena_autonomy/partial_observation.py')},
        unknown_angles_deg=[float(np.degrees(sample.angle_min+i*sample.angle_increment)) for i in unknown],rows=rows)
    with (OUT/'run8_latch_probe.json').open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result))


if __name__=='__main__':main()

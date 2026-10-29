"""재관측 국소 후보를 고정 run2 입력에서 분리 비교한다. 새 주행/상태 전체 재생 아님."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.surface_reobservation import supported_surface
import arena_autonomy.surface_anchor_trial as anchor_module
from arena_autonomy.partial_observation import short_observed_surface


def main():
    source=BASE/'run2_reobservation_fixture_v1.json'
    output=BASE/'surface_candidate_probe_v1.json'
    if output.exists():raise FileExistsError(output)
    raw=source.read_bytes();fixture=json.loads(raw)
    config=MotionConfig(**fixture['motion_config'])
    data,scan,history=restore_call(fixture['original_input'],config)
    source_time=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
    points,ids,_=history.project(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,
        scan.range_max,source_time,scan.time_increment,data['reference_s'],'both',True)
    keep=np.linalg.norm(points,axis=1)<=3;old=points[keep];old_ids=ids[keep]
    rows=[]
    for name,matcher in [('legacy',short_observed_surface),('four_returns',supported_surface)]:
        anchor_module.short_observed_surface=matcher
        probe=anchor_module.SurfaceAnchorTrial()
        probe.start(old[fixture['retained_indices']],old,old_ids,data['reference_s'],source_time)
        outputs=[]
        for inputs in fixture['inputs']:
            d,s,h=restore_call(inputs,config)
            first=s.header.stamp.sec+s.header.stamp.nanosec*1e-9
            p,i,_=h.project(s.ranges,s.angle_min,s.angle_increment,s.range_min,s.range_max,
                first,s.time_increment,d['reference_s'],'both',True)
            result=probe.observe(p,i,h,d['reference_s'],first)
            if isinstance(result.get('fit'),dict):result['fit'].pop('processing_ms',None)
            outputs.append(dict(t=d['reference_s'],result=result))
        rows.append(dict(name=name,outputs=outputs))
    anchor_module.short_observed_surface=short_observed_surface
    assert source.read_bytes()==raw
    output.write_text(json.dumps(freeze(dict(scope='isolated probe; no motion or command proof',
        input_sha256=hashlib.sha256(raw).hexdigest(),input_unchanged=True,results=rows)),indent=2),encoding='utf-8')
    print(json.dumps([dict(name=r['name'],first=[dict(t=x['t'],reason=x['result']['reason'],matched=x['result']['matched']) for x in r['outputs'][:4]]) for r in rows]))


if __name__=='__main__':main()

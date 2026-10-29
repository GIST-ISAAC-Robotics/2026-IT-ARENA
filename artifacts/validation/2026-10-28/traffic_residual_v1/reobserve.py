"""잔여 래치 근거가 후속 실측 반환에 다시 나타나는지만 진단한다."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig,MotionUnavailable
from arena_autonomy.scene_motion import motion_scene,transport_points
from arena_autonomy.partial_observation import short_observed_surface

def main():
    output=BASE/'reobservations.json'
    if output.exists():raise FileExistsError(output)
    original=BASE.parent/'timed_latch_v1/policy_replay_v1/report.json'
    latch=json.loads(original.read_text())['first_latch']['latch_evidence']
    points=np.asarray([w['previous_point_now_xy'] for w in latch['witnesses']])
    trace=ROOT/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course/follow_decisions.jsonl'
    samples=[];last_source=None;run=0;first=None;unavailable=0
    with trace.open() as stream:
        header=json.loads(next(stream))['data'];motion=MotionConfig(**header['motion_config'])
        for line in stream:
            row=json.loads(line)
            if row['kind']!='call':continue
            d=row['data']['input'];now=d['reference_s']
            if now<latch['reference_s']:continue
            if now>latch['reference_s']+1.:break
            d,scan,h=restore_call(d,motion)
            source=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            if source==last_source:continue
            last_source=source
            try:
                carried=transport_points(points,h,latch['reference_s'],now)
                scene=motion_scene(scan,h,now,timing_verified=d['timing_verified'],epoch=d['epoch'],
                                   rear_half_angle_deg=d['rear_half_angle_deg'])
            except MotionUnavailable:
                unavailable+=1;run=0;continue
            rays=[r for r in scene.rays if r.status=='return']
            current=np.asarray([r.hit_xy_m for r in rays]);indices=np.array([r.source_index for r in rays])
            distances=np.min(np.linalg.norm(carried[:,None]-current[None],axis=2),axis=1)
            observed=[bool(distance<=.02 or (distance<=.04 and short_observed_surface(p,current,indices) is not None))
                      for p,distance in zip(carried,distances)]
            run=run+1 if all(observed) else 0
            if run>=2 and first is None:first=now
            samples.append(dict(reference_s=now,scan_source_s=source,nearest_distances_m=distances.tolist(),
                reobserved=observed,consecutive_fresh_sources=run,
                current_unknown_count=sum(r.status=='no_return' for r in scene.rays)))
    inputs=[Path(__file__),original,trace,ROOT/'src/arena_autonomy/arena_autonomy/partial_observation.py']
    result=dict(scope='stored-input geometric reacquisition diagnostic only; no release, identity or safety proof',
        first_two_source_reobservation_s=first,motion_unavailable=unavailable,samples=samples,
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})
    output.write_text(json.dumps(freeze(result),indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'}))

if __name__=='__main__':main()

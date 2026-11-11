"""분리된 좁은 공백을 둘러싼 더 큰 단일 쐐기의 기하만 비교한다. 주행 미연결."""
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.scan_input import scan_angles,scan_times,scan_max_gap
from arena_autonomy.scene_motion import relative_poses
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig


def envelope(scan,history,now,speed,steering,previous,wheelbase=.135,length=.2,width=.15):
    result=dict(eligible=False,scope='geometry_only_not_command_permission',unknown_filled=False)
    if not all(math.isfinite(x) for x in (now,speed,steering,previous)) or not 0<=speed<=.6:
        return dict(result,reason='speed_or_motion_outside_existing_metric')
    stamps=scan_times(scan);age=now-stamps[0]
    if not 0<=age<.3:return dict(result,reason='source_age')
    angles=(scan_angles(scan)+history.config.lidar_yaw_rad+math.pi)%math.tau-math.pi
    values=np.asarray(scan.ranges);order=np.argsort(angles)
    front=order[np.abs(angles[order])<=math.radians(100)+1e-6]
    valid=np.isfinite(values)&(values>=scan.range_min)&(values<=scan.range_max)
    ids=np.flatnonzero(~valid[front])
    if not len(ids):return dict(result,reason='no_missing')
    runs=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
    if len(runs)<2:return dict(result,reason='not_multiple_runs')
    if ids[0]==0 or ids[-1]==len(front)-1:return dict(result,reason='unbounded')
    missing=front[ids]
    if not np.all(np.isposinf(values[missing])):return dict(result,reason='not_positive_inf')
    # 사이의 관측점과 바깥 반환을 지우지 않고, 더 큰 포괄 쐐기만 잰다.
    covered=front[ids[0]-1:ids[-1]+2]
    known=covered[valid[covered]]
    required=max(1.,.16+speed*(age+.1)+speed*speed/4)
    if np.any(values[known]<required):return dict(result,reason='near_observed_point')
    current=(angles[covered]+relative_poses(history,stamps[covered],now)[:,2]+math.pi)%math.tau-math.pi
    if np.any(abs(current)>math.radians(100)+1e-6):return dict(result,reason='rotation_outside_front')
    source_span=float(angles[covered[-1]]-angles[covered[0]])
    current_span=float(max(current)-min(current))
    step=scan_max_gap(scan)
    # 기존 단일 결측6도에 양옆 반환까지의 한 셀 간격을 더한 상한이다.
    span=max(source_span,current_span)
    if span>math.radians(6)+step+1e-7:return dict(result,reason='enclosing_wedge_too_wide')
    radius=max(math.hypot(length/2+.05,width/2+.05)+math.hypot(history.config.lidar_x_m,history.config.lidar_y_m)
        +(1+max(abs(math.tan(steering)),abs(math.tan(previous)))/2)*(speed*(age+.1)+speed*speed/4+.06),
        .16+speed*(age+.1)+speed*speed/4)
    chord=2*radius*math.sin(span/2)
    return dict(result,eligible=chord<=.05,reason='bounded_enclosing_wedge' if chord<=.05 else 'physical_width_exceeded',
        runs=len(runs),missing_indices=missing.tolist(),covered_indices=covered.tolist(),
        source_span_deg=math.degrees(source_span),current_span_deg=math.degrees(current_span),
        checked_speed_mps=speed,radius_m=radius,hidden_width_m=chord,maximum_hidden_width_m=.05,
        observed_internal_points=len(known)-2,minimum_observed_range_m=float(min(values[known])),
        residual_hidden_object_risk=True)


def main():
    source=A.parent/'forward_gap_scope_r109_v1/two_actor_run1/run1/follow_decisions.jsonl'
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    rows=[json.loads(s) for s in source.open(encoding='utf8')]
    motion=MotionConfig(**rows[0]['data']['motion_config']);captures=[]
    for row in rows:
        if row['kind']!='call':continue
        data=row['data'];normal=data['output'][2].get('normal',{})
        partial=normal.get('partial_observation',{})
        if partial.get('reason')!='forward_gap_too_wide':continue
        inputs,scan,history=restore_call(data['input'],motion)
        trial=envelope(scan,history,inputs['reference_s'],max(.2,abs(inputs['speed_mps'])),
            data['output'][1],inputs['previous_steering_rad'])
        captures.append(dict(seq=row['seq'],reference_s=inputs['reference_s'],
            original_output=data['output'][:2],original_partial=partial,candidate_geometry=trial))
    assert hashlib.sha256(source.read_bytes()).hexdigest()==digest
    report=freeze(dict(source_sha256=digest,captures=captures,
        eligible=sum(c['candidate_geometry']['eligible'] for c in captures),total=len(captures),
        scope='saved sensor geometry; no controller mutation, path approval, independent guard or driving test'))
    with (A/'geometry.json').open('x',encoding='utf8') as stream:json.dump(report,stream,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k!='captures'}))


if __name__=='__main__':main()

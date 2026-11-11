"""r109 마지막 공백의 실제 그룹/저속 물리폭 분석. 정책/명령을 바꾸지 않는다."""
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.scan_input import scan_angles,scan_cell_widths,scan_times,scan_max_gap
from arena_autonomy.scene_motion import relative_poses
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision


def main():
    source=A/'two_actor_run1/run1/follow_decisions.jsonl'
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    rows=[json.loads(s) for s in source.open(encoding='utf8')]
    follow,motion=make_controller(rows[0]['data'])
    original=PartialObservationPolicy.evaluate
    active={};captures=[];mismatch=[];command_changes=[]
    def inspect(self,scan,points,history,now,speed,steering=0.,previous_steering=0.,
                wheelbase=.135,length=.2,width=.15):
        result=original(self,scan,points,history,now,speed,steering,previous_steering,wheelbase,length,width)
        if active['seq'] not in (924,927,930,950) or result['reason']!='forward_gap_too_wide':return result
        values=np.asarray(scan.ranges)
        angles=(scan_angles(scan)+history.config.lidar_yaw_rad+math.pi)%math.tau-math.pi
        front=np.argsort(angles);front=front[np.abs(angles[front])<=math.radians(100)+1e-6]
        missing=~(np.isfinite(values)&(values>=scan.range_min)&(values<=scan.range_max))
        excused=getattr(self,'structure_excused',None)
        if excused is not None:missing&=~np.asarray(excused,dtype=bool)
        ids=np.flatnonzero(missing[front]);runs=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
        cells=scan_cell_widths(scan);stamps=scan_times(scan);step=scan_max_gap(scan)
        age=now-stamps[0]
        # 기존 단일 공백 정책과 같은 식을 분석만 한다. 다른 정책 허가로 전달하지 않는다.
        radius=max(math.hypot(length/2+.05,width/2+.05)+math.hypot(history.config.lidar_x_m,history.config.lidar_y_m)
            +(1+max(abs(math.tan(steering)),abs(math.tan(previous_steering)))/2)*(speed*(age+.1)+speed*speed/4+.06),
            .16+speed*(age+.1)+speed*speed/4)
        groups=[]
        for run in runs:
            indices=front[run];poses=relative_poses(history,stamps[indices],now)
            current=(angles[indices]+poses[:,2]+math.pi)%math.tau-math.pi
            width_rad=float(sum(cells[indices]));neighbors=front[[run[0]-1,run[-1]+1]] if run[0]>0 and run[-1]<len(front)-1 else []
            groups.append(dict(indices=indices.tolist(),source_deg=np.degrees(angles[indices]).tolist(),
                current_deg=np.degrees(current).tolist(),width_deg=math.degrees(width_rad),
                neighbor_ranges_m=[float(values[i]) for i in neighbors],
                positive_inf_only=bool(np.all(np.isposinf(values[indices]))),
                chord_at_existing_radius_m=2*radius*math.sin((width_rad+step)/2)))
        captures.append(dict(seq=active['seq'],reference_s=now,speed_mps=speed,steering_rad=steering,
            radius_m=radius,groups=groups,total_chord_m=sum(g['chord_at_existing_radius_m'] for g in groups),
            original=result,scope='geometry_only_not_policy_admission'))
        return result
    PartialObservationPolicy.evaluate=inspect
    try:
        for row in rows[1:]:
            if row['kind']=='reset':follow.reset()
            elif row['kind']=='call':
                active['seq']=row['seq'];out=freeze(replay_call(follow,row['data']['input'],motion))
                if not equal_output(comparable_decision(out),comparable_decision(row['data']['output'])):mismatch.append(row['seq'])
                if not equal_output(out[:2],row['data']['output'][:2]):command_changes.append(row['seq'])
    finally:PartialObservationPolicy.evaluate=original
    assert hashlib.sha256(source.read_bytes()).hexdigest()==digest
    report=freeze(dict(source_sha256=digest,mismatches=mismatch,command_changes=command_changes,captures=captures))
    with (A/'final_gap_audit_wsl.json').open('x',encoding='utf8') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(dict(mismatches=len(mismatch),command_changes=len(command_changes),captures=[
        {k:v for k,v in c.items() if k!='original'} for c in captures])))


if __name__=='__main__':main()

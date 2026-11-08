"""입구 통과·노면·접촉·STOP을 평가한다. 정답은 제어기에 주지 않는다."""
import bisect,hashlib,json,math,sys
from pathlib import Path
import numpy as np
from surface_contact_eval import SurfaceMap, VehicleGeometry, evaluate_pose
from arena_autonomy.follow_trace import (restore_call,replay_call,freeze,equal_output,comparable_decision)
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.p22_trial import P22Entry

A=Path(__file__).resolve().parent;D=A/'run1'
load=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    report=load(D/'report.json');source=D/'follow_decisions.jsonl';before=sha(source)
    first=None;calls=0;mismatch=[];footer=None;epochs=[]
    reasons={};last=None
    for line in source.open():
        event=json.loads(line);k=event['kind'];d=event['data']
        if k=='header':
            assert d['schema']==6 and d['controller_mode']=='p22_entry_trial'
            mc=MotionConfig(**d['motion_config']);f=P22Entry(LocalFollow(FollowConfig(**d['follow_config'])))
        elif k=='p22_frame':f.frame(**d)
        elif k=='reset':f.reset(force=d['reason']=='autonomy_reset')
        elif k=='call':
            calls+=1;out=replay_call(f,d['input'],mc)
            if not equal_output(comparable_decision(freeze(out)),comparable_decision(d['output'])):
                mismatch.append(d['input']['reference_s'])
            detail=d['output'][2];state=detail.get('p22_trial') or {}
            if state.get('first_command_admitted') and first is None:first=d['input']['reference_s']
            if first is not None:reasons[detail.get('reason')]=reasons.get(detail.get('reason'),0)+1
            last=detail
        elif k=='footer':footer=d
    poses=[json.loads(l) for l in (D/'ego_poses.jsonl').open()]
    trace=load(D/'trace.json');times=[t['time'] for t in trace]
    surface=SurfaceMap.from_world(D/'runtime_world.sdf');g=VehicleGeometry()
    maxoff=0;contact=0;entered=[];sampled=0;wallgap=math.inf
    from shapely.geometry import Polygon
    # 평가 등록: 입구 x=5.635, y=4.2..4.8. 서향으로 차체 전체가 통과했는지 검사.
    last_eval=-float("inf")
    for t,x,y,yaw in poses:
        if first is None or t<first or t-last_eval<.0195:continue
        last_eval=t
        i=max(0,min(len(trace)-1,bisect.bisect_right(times,t)-1))
        steer=trace[i]['autonomy']['steering_command_rad']
        # 실제 바퀴각 대체값이 아닌 명령 조향 기반 접지면 평가로 한정한다.
        e=evaluate_pose(surface,x,y,yaw,steer,g)
        maxoff=max(maxoff,e['off_counts']['any']);sampled+=1
        c,s=math.cos(yaw),math.sin(yaw)
        corners=[(x+c*dx-s*dy,y+s*dx+c*dy) for dx,dy in ((.1,.075),(.1,-.075),(-.1,-.075),(-.1,.075))]
        shape=Polygon(corners);contact+=int(shape.intersection(surface.walls).area>1e-6)
        wallgap=min(wallgap,shape.distance(surface.walls))
        err=abs(math.atan2(math.sin(yaw-math.pi),math.cos(yaw-math.pi)))
        if max(v[0] for v in corners)<5.635 and min(v[1] for v in corners)>4.2 and max(v[1] for v in corners)<4.8 and err<math.radians(20):
            entered.append(t)
    integrity=load(A/'postflight_integrity.json')
    out=dict(scope='limited P22 entry only; not exit merge or lap benefit',
        admitted_s=first,calls=calls,replay_mismatches=mismatch[:20],replay_mismatch_count=len(mismatch),
        input_unchanged=sha(source)==before,footer=footer,active_reason_counts=reasons,
        whole_body_entered=bool(entered),first_whole_body_inside_s=entered[0] if entered else None,
        entry_pose_samples=sampled,wall_overlap_samples=contact,min_wall_gap_m=wallgap if sampled else None,
        max_offroad_wheels=maxoff,road_scope='50Hz sampled wheel patches using commanded steering, not measured joint angles',
        stop_stable=report.get('stop_stable'),actuation_passed=report.get('actuation_passed'),
        cleanup_stop=report.get('failure_cleanup_stop'),shutdown_clean=report.get('shutdown_clean'),
        validator_passed=report.get('passed'),validator_error=report.get('error'),inputs=integrity,
        limited_entry_passed=bool(entered) and contact==0 and maxoff<=2 and report.get('stop_stable') is True
            and report.get('actuation_passed') is True and report.get('shutdown_clean') is True
            and not mismatch and not integrity['changed'])
    (A/'live_summary.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf8')
    print(json.dumps(out,indent=2))
if __name__=='__main__':main()

"""r50 단독 입출구 평가. 정답 자세는 실행 뒤 평가에만 사용한다."""
import bisect
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from shapely.geometry import Polygon
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.p22_route_trial import P22Route
from surface_contact_eval import SurfaceMap,VehicleGeometry,evaluate_pose

A=Path(__file__).resolve().parent
load=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def outline(x,y,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return [(x+c*dx-s*dy,y+s*dx+c*dy) for dx,dy in ((.1,.075),(.1,-.075),(-.1,-.075),(-.1,.075))]


def heading_near(actual,desired):
    return abs(math.atan2(math.sin(actual-desired),math.cos(actual-desired)))<math.radians(20)


def trace_integrity(footer,event_count):
    return bool(footer and footer.get('accepted')==footer.get('written')==event_count-1
                and all(footer.get(k)==0 for k in ('dropped','failed','pending','capture_errors'))
                and footer.get('worker_alive') is False)


def motion_metrics(samples,merged):
    slow=longest=progress=0.
    for a,b in zip(samples,samples[1:]):
        dt=b[0]-a[0]
        if dt<=0 or dt>.05:raise ValueError('pose sample gap or reorder')
        speed=math.hypot(b[1]-a[1],b[2]-a[2])/dt
        slow=slow+dt if speed<.05 else 0.
        longest=max(longest,slow)
        if merged is not None and a[0]>=merged:
            progress+=max(0.,math.cos(a[3])*(b[1]-a[1])+math.sin(a[3])*(b[2]-a[2]))
    return longest,progress


def main():
    d=A/'run1';report=load(d/'report.json');source=d/'follow_decisions.jsonl';digest=sha(source)
    calls=count=0;mismatch=[];sequence_errors=[];first=exit_at=merged=complete=None;footer=None
    phase_counts={};reason_counts={}
    for line in source.open():
        e=json.loads(line);k=e['kind'];v=e['data']
        if e.get('seq')!=count or footer is not None:sequence_errors.append(count)
        count+=1
        if k=='header':
            assert count==1 and v['schema']==7 and v['controller_mode']=='p22_route_trial'
            mc=MotionConfig(**v['motion_config']);f=P22Route(LocalFollow(FollowConfig(**v['follow_config'])))
        elif k=='p22_frame':f.frame(**v)
        elif k=='reset':f.reset(force=v.get('force',v['reason'] in ('autonomy_reset','legacy_reset')))
        elif k=='call':
            calls+=1;t=v['input']['reference_s'];out=replay_call(f,v['input'],mc)
            if not equal_output(comparable_decision(freeze(out)),comparable_decision(v['output'])):mismatch.append(t)
            q=v['output'][2];p=q.get('p22_trial',{})
            if p.get('first_command_admitted') and first is None:first=t
            if p.get('phase')=='exit' and exit_at is None:exit_at=t
            if p.get('phase')=='main' and merged is None:merged=t
            if p.get('route_complete') and complete is None:complete=t
            if first is not None:
                phase_counts[p.get('phase','missing')]=phase_counts.get(p.get('phase','missing'),0)+1
                reason_counts[q.get('reason')]=reason_counts.get(q.get('reason'),0)+1
        elif k=='footer':footer=v
        else:raise ValueError('unexpected trace event '+k)
    trace=load(d/'trace.json');times=[r['time'] for r in trace]
    end=(report.get('p22_route_end') or report.get('p22_entry_end') or {}).get('time_s',times[-1])
    poses=[]
    for line in (d/'ego_poses.jsonl').open():
        p=json.loads(line)
        if first is not None and first<=p[0]<=end and (not poses or p[0]-poses[-1][0]>=.0195):poses.append(p)
    surface=SurfaceMap.from_world(d/'runtime_world.sdf');geometry=VehicleGeometry()
    entered=[];exited=[];maxoff=overlap=0;gap=math.inf
    for t,x,y,yaw in poses:
        i=max(0,min(len(trace)-1,bisect.bisect_right(times,t)-1));steer=trace[i]['autonomy']['steering_command_rad']
        evaluation=evaluate_pose(surface,x,y,yaw,steer,geometry)
        maxoff=max(maxoff,evaluation['off_counts']['any'])
        corners=outline(x,y,yaw);body=Polygon(corners)
        overlap+=int(body.intersection(surface.walls).area>1e-6);gap=min(gap,body.distance(surface.walls))
        xs,ys=zip(*corners)
        if max(xs)<5.635 and min(ys)>4.2 and max(ys)<4.8 and heading_near(yaw,math.pi):entered.append(t)
        if min(xs)>3.81 and max(xs)<4.81 and min(ys)>4.8 and heading_near(yaw,math.pi/2):exited.append(t)
    longest,progress=motion_metrics(poses,merged)
    integrity=load(A/'postflight_integrity.json')
    checks=dict(entry_and_exit=bool(entered and exited),ordered_states=all(v is not None for v in (first,exit_at,merged,complete))
                and first<exit_at<merged<complete,post_merge_progress=progress>=1.,no_contact=overlap==0 and report.get('collision_samples')==0,
                max_two_wheels=maxoff<=2,continuous_progress=longest<=.5,
                stop=report.get('stop_stable') is True and report.get('actuation_passed') is True,
                shutdown=report.get('shutdown_clean') is True,sensor_only=report.get('sensor_only_inputs') is True
                and report.get('safety_sensor_only_inputs') is True,
                no_false_start=report.get('false_start') is False,replay=calls>0 and not mismatch,
                trace_integrity=not sequence_errors and trace_integrity(footer,count),
                input_unchanged=sha(source)==digest and not integrity['changed'],pose_samples=len(poses)>10)
    out=dict(scope='one limited solo entry-exit-main return; not traffic merge or whole lap',checks=checks,
             limited_route_passed=all(checks.values()),first_admitted_s=first,exit_selected_s=exit_at,main_handoff_s=merged,
             controller_complete_s=complete,evaluation_end_s=end,first_body_inside_s=entered[0] if entered else None,
             first_body_exit_s=exited[0] if exited else None,actual_post_merge_progress_m=progress,
             longest_slow_s=longest,min_wall_gap_m=gap if poses else None,max_offroad_wheels=maxoff,wall_overlap_samples=overlap,
             pose_samples=len(poses),calls=calls,replay_mismatch_count=len(mismatch),replay_mismatches=mismatch[:10],
             phase_counts=phase_counts,reason_counts=reason_counts,footer=footer,inputs=integrity,input_sha256=digest,
             raw_validator_passed=report.get('passed'),raw_validator_error=report.get('error'),
             metrics_scope='50Hz world pose samples until request, <.05m/s finite-difference speed; main distance is forward projection of pose increments, no interpolation; road wheel patches use commanded steering, not measured joint angles')
    (A/'live_summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(out,ensure_ascii=False,indent=2))
    # 결과 실패도 원시 평가를 온전히 기록한 완료다. 실제 판정은 limited_route_passed.


if __name__=='__main__':main()

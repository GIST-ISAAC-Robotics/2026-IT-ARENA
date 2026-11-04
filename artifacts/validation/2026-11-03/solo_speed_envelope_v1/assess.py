"""원본 판단 뒤 속도 증분만 계산한다. 제어기 전체·부모·독립 보호의 폐루프 재실행이 아니다."""
import argparse
import csv
import hashlib
import json
import math
import sys
from bisect import bisect_right
from collections import Counter
from pathlib import Path
from time import perf_counter

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from straight_budget import eligible, estimate

SOURCES=dict(prior='2026-11-02/solo_alternative_speed_v1',post_free='2026-11-03/solo_post_free_v1')


def digest(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f,'sha256').hexdigest()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('label',choices=SOURCES)
    args=parser.parse_args()
    p=HERE.parents[1]/SOURCES[args.label]/'run1'
    report=json.loads((p/'report.json').read_bytes())
    green=report['start_time_sim_s']
    feedback=json.loads((p/'actuation_trace.json').read_bytes())
    stop=next(r['now_us']*1e-6 for r in feedback if r['reason']=='software_stop')
    poses=json.loads((p/'trace.json').read_bytes())
    pose_t=[s['time'] for s in poses]
    # 1.5 m/s²는 두 입력에 쓰인 부모 출력을 재구성한 기존 검증 값이다. 가속 설정은 바꾸지 않는다.
    prev=[0.,0.];rows=[];reasons=Counter();cost=[];sequence=0;footer=None
    for event in map(json.loads,(p/'follow_decisions.jsonl').open(encoding='utf8')):
        assert event['seq']==sequence
        sequence+=1
        data=event['data']
        if event['kind']=='header':
            cfg=FollowConfig(**data['follow_config']);motion=MotionConfig(**data['motion_config'])
            assert cfg.solo_route_only and not cfg.ray_wall_continuity
        elif event['kind']=='reset':
            prev=[0.,0.]
        elif event['kind']=='footer':
            footer=data
        elif event['kind']=='call':
            i=data['input'];t=i['reference_s']
            if t>=stop:
                continue
            old,steer,details=output=data['output']
            dt=i['dt_s'];old_steer=i['previous_steering_rad'];meas=i['speed_mps']
            if eligible(output,old_steer,meas,cfg):
                i,scan,history=restore_call(i,motion)
                scene=motion_scene(scan,history,t,timing_verified=i['timing_verified'],
                                   epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
                then=perf_counter()
                new,why=estimate(output,scene,old_steer,meas,cfg)
                cost.append(1000*(perf_counter()-then))
            else:
                new,why=old,'original_state'
            assert math.isfinite(new) and new>=old-1e-9 and (new==old or why=='straight_budget')
            curve=abs(math.tan(steer)/cfg.wheelbase_m)
            curve_cap=math.sqrt(cfg.lateral_acceleration_mps2/curve) if curve>1e-6 else math.inf
            prev=[min(v,curve_cap,last+1.5*dt) for last,v in zip(prev,[old,new])]
            if t>=green:
                # 이하 progress는 집계표 전용이며 후보 함수 입력에 없다.
                s=poses[max(0,bisect_right(pose_t,t)-1)]['progress_m']
                rows.append(dict(t=t,dt=dt,s=s,old=old,new=new,old_ramp=prev[0],new_ramp=prev[1],
                                 steering=steer,partial=details.get('partial_observation',{}).get('reason'),why=why))
                reasons[why]+=1
    assert footer and footer['accepted']==footer['written']==sequence-1
    assert not any(footer.get(k) for k in ('failed','dropped','pending','capture_errors','worker_alive'))
    assert not any(r['new']!=r['old'] and r['partial']!='fully_observed' for r in rows)
    result=dict(input_sha256={name:digest(p/name) for name in ('follow_decisions.jsonl','actuation_trace.json','report.json','trace.json')},
                analysis_sha256={name:digest(HERE/name) for name in ('straight_budget.py','assess.py')},
                window_s=[green,stop],calls=len(rows),reasons=dict(reasons),
                changed=sum(r['new']!=r['old'] for r in rows),
                changed_at_42_to_44_m=sum(r['new']!=r['old'] and 42<=r['s']<44 for r in rows),
                first_changed_s=next((r['t'] for r in rows if r['new']!=r['old']),None),
                zero_speed_calls=[sum(r[k]<=0 for r in rows) for k in ('old','new')],
                max_local_mps=[max(r[k] for r in rows) for k in ('old','new')],
                max_parent_mps=[max(r[k] for r in rows) for k in ('old_ramp','new_ramp')],
                above_2p5_parent_sample_hold_s=[sum(r['dt'] for r in rows if r[k]>=2.5) for k in ('old_ramp','new_ramp')],
                signed_parent_integral_m=sum(r['dt']*(r['new_ramp']-r['old_ramp']) for r in rows),
                analysis_cost_ms=dict(evaluated=len(cost),p50=float(np.median(cost)),p95=float(np.percentile(cost,95)),maximum=max(cost)),
                scope='기존 주행의 실제 스캔·기존 판단 기록 위에서 적용 가능성과 부모 증속률 근사만 확인. 비용은 기존 판단 뒤 추가 연산이며 입력 복원 제외. 같은 입력에서 조향을 바꾸는 출력 경로가 없다. 차체 속도나 정지 해제의 대체 궤적은 측정하지 않았다.')
    with (HERE/(args.label+'_calls.csv')).open('x',newline='',encoding='utf8') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    with (HERE/(args.label+'.json')).open('x',encoding='utf8') as f:
        json.dump(result,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
    print(args.label,{k:result[k] for k in ('calls','changed','changed_at_42_to_44_m','max_parent_mps','above_2p5_parent_sample_hold_s','analysis_cost_ms')})


if __name__=='__main__':
    main()

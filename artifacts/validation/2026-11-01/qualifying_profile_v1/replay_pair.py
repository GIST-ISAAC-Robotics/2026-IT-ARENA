"""같은 10 km/h 기준 랩에 코너 보완과 1.5 증속을 함께 적용한 발행 명령을 비교한다."""
from collections import Counter
from dataclasses import replace
from hashlib import sha256
import json
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionConfig

HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'corner_approach_v1'


def replay():
    run=OLD/'solo_10kmh';report=json.loads((run/'report.json').read_bytes())
    metrics=json.loads((OLD/'solo_10kmh_metrics.json').read_bytes())
    start=metrics['first_green_sample_s'];end=start+metrics['lap_s_from_first_green_sample']
    ref=json.loads((HERE/'reference_input.json').read_bytes())
    if not report['passed'] or sha256((run/'report.json').read_bytes()).hexdigest()!=ref['report_sha256']:
        raise ValueError('기준 실행 보고서 불일치')
    digest=sha256();rows=[];prev=np.zeros(3);rates=np.array([1.,1.5,1.5]);footer=None;last_seq=-1
    tprev=None;counts=Counter();eligible=set();ref_stamps={}
    with (run/'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            digest.update(line);r=json.loads(line);d=r['data']
            if r['seq']!=last_seq+1:raise ValueError('판단 기록 순번 누락')
            last_seq=r['seq']
            if r['kind']=='footer':footer=d
            if r['kind']=='header':
                cfg=FollowConfig(**d['follow_config'])
                assert np.isclose(cfg.open_route_speed_mps,10/3.6,atol=.001) and cfg.blend_open_speed_by_length and not cfg.preview_turn_speed
                candidate=LocalFollow(replace(cfg,preview_turn_speed=True));motion=MotionConfig(**d['motion_config'])
            if r['kind']=='reset':candidate.reset();prev[:]=0.;tprev=None
            if r['kind']!='call':continue
            t=d['input']['reference_s']
            if t>end:continue
            v,angle,det=replay_call(candidate,d['input'],motion)
            old,steer,_=d['output']
            if tprev is not None and t-tprev>.03:prev[:]=0.
            prev=np.minimum([old,old,v],prev+rates/50.)
            if start<=t<=end:
                rows.append([t,*prev]);ref_stamps[round(t,6)]=prev[0]
                counts['calls']+=1;counts['steering_changed']+=abs(angle-steer)>1e-8
                counts['zeros_changed']+=(old==0)!=(v==0);counts['lower_plan']+=v<old-1e-8
                if v>old+1e-8:counts['raised_plan']+=1;eligible.add(det['source_first_s'])
            tprev=t
    with (run/'trajectory.jsonl').open(encoding='utf-8') as f:
        errs=[]
        for row in map(json.loads,f):
            a=row.get('autonomy') or {};t=a.get('reference_stamp_s');v=a.get('speed_command_mps')
            if t is not None and v is not None and round(t,6) in ref_stamps:errs.append(abs(ref_stamps[round(t,6)]-v))
    if not errs or max(errs)>1e-7:raise ValueError('원본 발행 속도를 재현하지 못함')
    if (digest.hexdigest()!=ref['trace_sha256'] or not footer or footer['written']!=last_seq
            or any(footer[k] for k in ('failed','dropped','capture_errors','pending'))):
        raise ValueError('원본 판단 기록 불일치')
    a=np.asarray(rows);dt=np.diff(a[:,0]);dt=np.where((dt>0)&(dt<.03),dt,0.)
    result={}
    for k,name in enumerate(('original','rise_only','turn_and_rise'),1):
        speed=a[:-1,k]
        result[name]=dict(peak_command_mps=max(a[:,k]),above_2_mps_s=float(sum(dt[speed>=2.])),
            command_integral_above_original_m=float(np.dot(dt,speed-a[:-1,1])),
            changed_zero=int(sum((a[:,k]==0)!=(a[:,1]==0))))
    return dict(trace_sha256=digest.hexdigest(),baseline_exact_samples=len(errs),baseline_max_error=max(errs),
                counts=dict(counts),increased_scans=len(eligible),profiles=result,
                scope='같은 차량 경로 및 관측을 재사용한 명령 차이. 새 차체 경로, 랩 시간, 접지와는 다르다.')


if __name__=='__main__':
    result=replay()
    with (HERE/'combined_replay.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(result['counts'],result['profiles'])

"""코너 끝 0명령의 최초 원인을 저장 입력에서 스캔별로 분리한다. 런타임 변경 없음, 평가 전용.

호출: cause_trace.py <run-folder> <출력.json>
"""
from collections import Counter
from hashlib import sha256
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy import local_follow as module
from arena_autonomy.follow_trace import replay_call,restore_call,freeze,equal_output,comparable_decision
from arena_autonomy.lidar_motion import MotionConfig,MotionUnavailable
from arena_autonomy.scene_motion import relative_poses,transport_points

folder=Path(sys.argv[1]);output=Path(sys.argv[2])
metrics=json.loads((folder/'run1_metrics.json').read_bytes())
begin=metrics['first_green_sample_s'];end=begin+metrics['lap_s_from_first_green_sample']
rows=[r for r in map(json.loads,(folder/'run1/trajectory.jsonl').read_bytes().splitlines()) if r['light']=='green']
T=np.array([r['time'] for r in rows]);S=np.array([r['progress_m'] for r in rows])
V=np.array([math.hypot(r['dynamics']['truth_longitudinal_mps'],r['dynamics']['truth_lateral_mps']) for r in rows])
W=np.array([r['dynamics']['truth_yaw_rate_rad_s'] for r in rows])

events=[];last=-1;follow=None;previous_reason=None
memory_failures=Counter();displacements=[]
with (folder/'run1/follow_decisions.jsonl').open(encoding='utf-8') as stream:
    for line in stream:
        e=json.loads(line);d=e['data']
        if e['seq']!=last+1:raise ValueError('trace order')
        last=e['seq']
        if e['kind']=='header':
            config=module.FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
            follow=module.LocalFollow(config)
        elif e['kind']=='reset':follow.reset()
        elif e['kind']=='call':
            inp,scan,history=restore_call(d['input'],motion)
            now=inp['reference_s']
            state=dict(path=None if follow.path is None else len(follow.path),path_age=None if follow.path is None else now-follow.path_source,
                       held=follow.held,pending=follow.pending_count,release=follow.release_count)
            shift=near=yaw_step=lever=None
            if follow.path is not None:
                try:
                    moved=transport_points(follow.path,history,follow.path_ref,now)
                    change=np.linalg.norm(moved-follow.path,axis=1)
                    shift=float(np.max(change));near=float(change[np.argmin(np.linalg.norm(follow.path,axis=1))])
                    lever=float(np.max(np.linalg.norm(follow.path,axis=1)))
                    dx,dy,yaw=relative_poses(history,[follow.path_ref],now)[0]
                    yaw_step=float(yaw)
                except (ValueError,MotionUnavailable) as error:
                    shift='error:'+str(error)
            output_=replay_call(follow,d['input'],motion)
            if not equal_output(comparable_decision(freeze(output_)),comparable_decision(d['output'])):raise ValueError(f'replay mismatch {last}')
            v,a,o=output_
            if not begin<=now<=end:continue
            fresh='boundary_selection' in o or 'boundary_candidate_reason' in o
            c=o.get('corridor_candidate') or {}
            row=dict(seq=last,t=now,s=float(np.interp(now,T,S)),truth=float(np.interp(now,T,V)),yaw_rate=float(np.interp(now,T,W)),measured=inp['speed_mps'],
                     target=v,steering=a,reason=o.get('reason'),selection=o.get('boundary_selection'),fresh=fresh,accepted=o.get('fresh_boundary_accepted'),
                     candidate_reason=o.get('boundary_candidate_reason'),candidate_ahead=o.get('boundary_candidate_ahead_m'),bilateral=o.get('bilateral_reference'),
                     path_wall=o.get('path_wall'),from_memory=o.get('boundary_from_memory'),boundary_age=o.get('boundary_age_s'),ahead=o.get('observed_path_ahead_m'),
                     clearance=o.get('swept_clearance_m'),corridor=c.get('reason'),corridor_positive=c.get('positive'),corridor_selected=o.get('corridor_selected'),
                     before=state,shift=shift,near_shift=near,yaw_step=yaw_step,lever=lever,
                     partial=(o.get('partial_observation') or {}).get('reason'),partial_cap=(o.get('partial_observation') or {}).get('speed_cap_mps'),
                     underlying=o.get('underlying_reason'))
            events.append(row)
            if isinstance(shift,float):displacements.append(shift)
            if o.get('reason')=='follow_boundary_unconfirmed' and fresh:
                cause='ttl' if state['path_age'] is not None and state['path_age']>=config.memory_ttl_s else ('motion_limit' if isinstance(shift,float) and shift>config.memory_motion_limit_m else ('no_path' if state['path'] is None else 'other'))
                memory_failures[cause]+=1
stops=[];i=0
while i<len(events):
    if events[i]['target']<=.001 and events[i]['t']>begin+2:
        j=i
        while j+1<len(events) and (events[j+1]['target']<=.001 or events[j+1]['t']-events[j]['t']<.0): j+=1
        stops.append(dict(first_seq=events[i]['seq'],start=events[i]['t'],end=events[j]['t'],s=events[i]['s'],reason=events[i]['reason'],
                          scans=[{k:r[k] for k in ('t','s','truth','yaw_rate','target','reason','selection','accepted','candidate_reason','candidate_ahead','bilateral','path_wall','from_memory','boundary_age','ahead','clearance','corridor','corridor_selected','before','shift','near_shift','yaw_step','lever','partial','underlying')}
                                 for r in events[max(0,i-30):j+16] if r['fresh'] or r['target']<=.001]))
        i=j+1
    else:i+=1
result=dict(scope='저장 입력 재생에서 판단 직전 상태를 함께 기록한 진단. 출력은 기록과 일치를 확인했다.',run=folder.as_posix(),
    memory_motion_limit_m=config.memory_motion_limit_m,memory_ttl_s=config.memory_ttl_s,
    first_reasons=Counter(s['reason'] for s in stops),boundary_unconfirmed_causes=memory_failures,
    shift_percentiles={str(p):float(np.percentile(displacements,p)) for p in (50,90,99,100)},stops=stops,
    script_sha256=sha256(Path(__file__).read_bytes()).hexdigest())
output.write_text(json.dumps(freeze(result),ensure_ascii=False,indent=1),encoding='utf-8',newline='\n')
print(json.dumps({k:result[k] for k in ('first_reasons','boundary_unconfirmed_causes','shift_percentiles')},ensure_ascii=False))
for s in stops:
    print(f"--- stop t={s['start']:.2f} s={s['s']:.2f} dur={s['end']-s['start']+.02:.2f} first={s['reason']}")
    for r in s['scans']:
        print(f"  {r['t']:.2f} s{r['s']:.2f} v{r['truth']:.2f} w{r['yaw_rate']:+.2f} T{r['target']:.2f} {str(r['reason'])[7:32]:25s} sel={str(r['selection'])[:22]:22s} acc={str(r['accepted'])[:1]} cand={str(r['candidate_reason'])[:12]} ahead={r['candidate_ahead'] if r['candidate_ahead'] is None else round(r['candidate_ahead'],2)} wall={r['path_wall']} mem={str(r['from_memory'])[:1]} shift={r['shift'] if not isinstance(r['shift'],float) else round(r['shift'],3)} near={r['near_shift'] if r['near_shift'] is None else round(r['near_shift'],3)} lev={r['lever'] if r['lever'] is None else round(r['lever'],2)} age={r['before']['path_age'] if r['before']['path_age'] is None else round(r['before']['path_age'],2)} hold={str(r['before']['held'])[:1]} cor={str(r['corridor'])[:28]} part={r['partial']}")

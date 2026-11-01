"""단독에서 검증한 2 m/s 코너 설정을 실제 교통 입력에 적용한 비교. 제어에는 연결하지 않는다."""
import argparse
from collections import Counter
from dataclasses import replace
from hashlib import sha256
import json
from pathlib import Path

from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig


def run(folder):
    report=json.loads((folder/'report.json').read_bytes())
    if not report['run_valid'] or not report['source_unchanged']:
        raise ValueError('유효한 실제 교통 입력 필요')
    window=report['case_window'];green=window['green_sim_s'];duration=window['duration_s']
    if report['case']=='first_curve':cuts=(0.,6.,13.,duration)
    elif report['case']=='stop_restart':cuts=(0.,4.,9.,duration)
    else:cuts=(0.,4.,duration)
    results=[Counter() for _ in cuts[:-1]];last=None
    digest=sha256();footer=None;reproduced=0
    with (folder/'follow_decisions.jsonl').open('rb') as f:
        for seq,line in enumerate(f):
            digest.update(line);e=json.loads(line);d=e['data']
            if seq!=e['seq'] or footer is not None:raise ValueError('판단 순번 또는 footer 오류')
            if e['kind']=='header':
                motion=MotionConfig(**d['motion_config']);config=FollowConfig(**d['follow_config'])
                if config.open_route_speed_mps!=1.5 or config.preview_turn_speed or config.blend_open_speed_by_length:
                    raise ValueError('예상한 1.5 m/s 교통 기준이 아님')
                params=HandoffExecutionConfig.from_mapping(d['execution_config'])
                def make(cfg):return BypassHandoffFollow(cfg,params,allow_simulation_trial=True)
                original=make(config)
                candidate=make(replace(config,open_route_speed_mps=2.,preview_turn_speed=True,blend_open_speed_by_length=True))
            elif e['kind']=='footer':footer=d
            elif e['kind']=='reset':original.reset();candidate.reset();last=None
            elif e['kind']=='call':
                inp=d['input'];t=inp['reference_s']-green
                if t>duration:continue
                old,angle,q=d['output'];v0,a0,base=replay_call(original,inp,motion)
                if max(abs(v0-old),abs(a0-angle))>1e-8 or base.get('handoff_state')!=q.get('handoff_state'):
                    raise ValueError(f'기준 출력 재현 실패 seq={seq}')
                reproduced+=1
                v,a,new=replay_call(candidate,inp,motion)
                if last and 0<=t-last[0]<=.05:
                    for j,(start,end) in enumerate(zip(cuts,cuts[1:])):
                        dt=max(0.,min(t,end)-max(start,last[0]))
                        results[j]['command_integral_difference_m']+=dt*last[1]
                        results[j]['covered_s']+=dt
                last=t,v-old
                for j,(start,end) in enumerate(zip(cuts,cuts[1:])):
                    if not start<=t<end:continue
                    st=results[j];st['calls']+=1
                    st['higher']+=int(v>old+1e-7);st['lower']+=int(v<old-1e-7)
                    st['new_zero']+=int(v<=.001<old);st['released_zero']+=int(old<=.001<v)
                    st['state_changes']+=int(new.get('handoff_state')!=q.get('handoff_state'))
                    st['steering_changes']+=int(abs(a-angle)>1e-8)
                    st['max_raise_mps']=max(st['max_raise_mps'],v-old)
                    st['max_cut_mps']=max(st['max_cut_mps'],old-v)
    if (not footer or footer['written']!=seq or any(footer[k] for k in ('failed','dropped','capture_errors','pending'))
        or digest.hexdigest()!=report['follow_decisions']['sha256']):
        raise ValueError('기록 전체성 확인 실패')
    return dict(case=report['case'],trace_sha256=digest.hexdigest(),baseline_commands_reproduced=reproduced,
                cuts_s=cuts,phases=[dict(x) for x in results],
                scope='같은 측정·속도·직전 실제 조향을 그대로 쓴 발행 전 속도·조향·전환. 실제 교통 진행이나 충돌 결과가 아니다.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('out',type=Path);a=p.parse_args()
    result=run(a.run)
    with a.out.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(result['case'],result['baseline_commands_reproduced'],result['phases'])

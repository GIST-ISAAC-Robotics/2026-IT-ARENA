"""별도 함수 전역 복사에서 주시거리만 교체. 실행 중인 설치 모듈/소스는 수정하지 않는다."""
from pathlib import Path
from functools import partial
import copy,json,types
import numpy as np
import arena_autonomy.bypass_entry_trial as entry_module
from arena_autonomy.follow_trace import restore_call,motion_scene,MotionConfig,FollowConfig
from adaptive_entry_preview import compare_shorter_previews,shorter_pursuit

A=Path(__file__).resolve().parent
d=json.loads((A.parent/'ops_traffic_centered_entry_v1/blocked_snapshot.json').read_bytes())
cfg=FollowConfig(**d['follow_config']);i,scan,history=restore_call(d['input'],MotionConfig(**d['motion_config']))
scene=motion_scene(scan,history,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
for line in (A.parent/'ops_traffic_centered_entry_v1/run1/follow_decisions.jsonl').open():
    row=json.loads(line)
    if row['kind']=='call' and row['data']['input']['reference_s']>=30:
        bypass=row['data']['output'][2]['bypass'];break
candidate=bypass['candidates'][0]
guide=dict(reference=bypass['reference'],path=np.asarray(d['path']),offset_m=candidate['offset_m'],transition_m=candidate['transition_m'],outer_side='right',stationary_outer_boundary=True)
entry=entry_module.EntryConfig(offsets_m=(0.,-.27,.27),transitions_m=(.6,.8,.9,1.2))
kwargs=dict(config=cfg,entry=entry,guide=guide,padding_provider=lambda *a:(np.asarray(d['padding']),dict(scope='frozen exact current padding')))

def compare_at(lookahead, previous, speed=0.):
    original=entry_module._compare_entries
    new_globals={**original.__globals__,'pursue_path':partial(shorter_pursuit,lookahead_m=lookahead)}
    detached=types.FunctionType(original.__code__,new_globals,original.__name__,original.__defaults__)
    detached.__kwdefaults__=original.__kwdefaults__
    result=detached(scene,scene.t_ref_s,speed,previous,.02,**kwargs)
    for row in result.get('candidates',[]):
        if 'path_points' in row:
            row['preview_unclipped_steering_rad']=shorter_pursuit(row['path_points'],speed,cfg,scene.t_ref_s-scene.source_first_s,lookahead_m=lookahead)[1]
    return result

previous=i['previous_steering_rad'];steps=[]
for step in range(8):
    primary=entry_module._compare_entries(scene,scene.t_ref_s,0.,previous,.02,**kwargs)
    selected=compare_shorter_previews(primary,compare_at,history=history,now=scene.t_ref_s,speed=0.,previous=previous,dt=.02)
    chosen=selected.get('selected');align=selected.get('preview_alignment')
    steps.append(dict(index=step,previous_rad=previous,primary=primary['reason'],chosen=chosen,align=align,preview=selected.get('adaptive_preview')))
    if chosen:break
    assert align and not align['new_motion_authorized']
    assert abs(align['requested_rad']-previous)<=.0600000001
    previous=align['requested_rad']
assert chosen and 0<chosen['speed_mps']<=.35
bad_history=copy.deepcopy(history);bad_history.wheels.times=[];bad_history.wheels.values=[]
checks={}
for label,speed,h in [('moving_shaft',.08129882628142206,history),('missing_shaft_history',0.,bad_history)]:
    p=entry_module._compare_entries(scene,scene.t_ref_s,speed,i['previous_steering_rad'],.02,**kwargs)
    out=compare_shorter_previews(p,lambda look,prev:compare_at(look,prev,speed),history=h,now=scene.t_ref_s,speed=speed,previous=i['previous_steering_rad'],dt=.02)
    checks[label]='preview_alignment' not in out
assert all(checks.values())
out=dict(scope='같은 고정 자세의 저장 센서 입력에서 조향 명령만 피드백한 가설. 실제 주행·서보 추종 증명이 아님.',steps=steps,negative_alignment_checks=checks,source_sha256=d['source_sha256'])
(A/'sequence_result.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8');print(json.dumps(out,ensure_ascii=False))

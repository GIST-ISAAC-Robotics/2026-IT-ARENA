"""정체 기록 재현. signed 대기 전의 원래 후보와 낮은 cap의 독립 상태 비교만 회수한다."""
import copy,hashlib,json,zipfile
from collections import Counter
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.bypass_corridor_trial import CorridorGuide
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
INPUT=BASE.parent/'g2_motion_pause_gazebo_v1/run1/follow_decisions.jsonl'


def sha(p):
    with p.open('rb') as s:return hashlib.file_digest(s,'sha256').hexdigest()


def compact(value,depth=0):
    if isinstance(value,dict):
        return {k:compact(v,depth+1) for k,v in value.items() if k not in
            ('path_points','ray_indices','source_indices','indices','members','endpoints_xy_m') and depth<8}
    if isinstance(value,(list,tuple)):
        return [compact(v,depth+1) for v in value[:4]]+([dict(omitted=len(value)-4)] if len(value)>4 else [])
    return value


def clone(guide):
    result=copy.copy(guide)
    shared=('config','entry','registered_entry','corridor_policy','reference_provider','motion_history')
    result.__dict__={k:(v if k in shared else copy.deepcopy(v)) for k,v in guide.__dict__.items() if k!='padding_provider'}
    result.padding_provider=result._padding
    return result


def main():
    output=BASE/'run1';output.mkdir(exist_ok=False)
    paths=[Path(__file__),* (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')]
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in paths};input_hash=sha(INPUT)
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in hashes:z.write(ROOT/p,p)
    original=CorridorGuide.evaluate;inner=[];calls=matched=0;footer=None;rows=[];reasons=Counter();negative=overspeed=0
    def observe(guide,scene,history,speed,previous,dt):
        before=clone(guide) if 24.<=scene.t_ref_s<=32. and speed>=0 else None
        result=original(guide,scene,history,speed,previous,dt)
        if before is not None:
            row=dict(t=scene.t_ref_s,speed=speed,previous=previous,cap=guide.entry.speed_cap_mps,
                reverse_wait=before.reverse_wait,confirmations=before.forward_confirmations,
                phase=before.phase,path_present=before.path is not None,result=compact(freeze(result)),alternatives=[])
            # 현재 같은 호출의 다른 cap을 조사할 뿐, 원래 guide 상태/출력에는 적용하지 않는다.
            if before.reverse_wait and result.get('reason')=='no_entry_command_candidate':
                for cap in (.25,.15,.08):
                    alt=clone(before);alt.entry=replace(alt.entry,speed_cap_mps=cap)
                    alt.registered_entry=replace(alt.registered_entry,speed_cap_mps=cap)
                    value=original(alt,scene,history,speed,previous,dt)
                    row['alternatives'].append(dict(cap=cap,result=compact(freeze(value))))
            inner.append(row)
        return result
    with patch.object(CorridorGuide,'evaluate',observe):
        for seq,line in enumerate(INPUT.open()):
            event=json.loads(line);assert event['seq']==seq and footer is None
            kind,d=event['kind'],event['data']
            if seq==0:
                assert kind=='header'
                follow=BypassHandoffFollow(FollowConfig(**d['follow_config']),HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
                motion=MotionConfig(**d['motion_config'])
            elif kind=='reset':follow.reset()
            elif kind=='call':
                t=d['input']['reference_s']
                if t>40.:continue
                result=freeze(replay_call(follow,d['input'],motion));calls+=1
                matched+=equal_output(comparable_decision(result),comparable_decision(d['output']))
                normal=result[2].get('normal') or {};bypass=result[2].get('bypass') or {}
                reasons[bypass.get('reason','none')]+=1
                negative+=normal.get('measured_speed_mps',0)<0 and result[0]>0
                overspeed+=bypass.get('reason')=='entry_overspeed_motion_hold'
                if 23.<=t<=32.:
                    rows.append(dict(t=t,command=result[:2],normal_reason=normal.get('reason'),speed=normal.get('measured_speed_mps'),
                        bypass_reason=bypass.get('reason'),underlying=bypass.get('underlying_reason'),
                        path_present=follow.bypass.guide.path is not None,phase=follow.bypass.guide.phase,
                        reverse_wait=follow.bypass.guide.reverse_wait,forward_confirmations=follow.bypass.guide.forward_confirmations))
            elif kind=='footer':footer=d
            else:raise ValueError(kind)
    assert footer and footer['accepted']==footer['written']==seq and not footer['worker_alive']
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    summary=dict(calls=calls,matched=matched,passed=calls==matched and sha(INPUT)==input_hash and all(sha(ROOT/p)==h for p,h in hashes.items()),
        input_sha256=input_hash,source_sha256=hashes,overspeed_holds=overspeed,positive_during_negative=negative,
        reasons=dict(reasons),inner_evaluations=len(inner),scope='same saved input, diagnostic alternatives never control the vehicle')
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    (output/'events.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
    (output/'inner.json').write_text(json.dumps(inner,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k!='source_sha256'}))
    return 0 if summary['passed'] else 1


if __name__=='__main__':raise SystemExit(main())

"""새 코스에서 기존 27 cm 편이 상한의 영향을 저장 센서로 격리한다. 제어기/기본값 변경 없음."""
from pathlib import Path
from dataclasses import asdict
import copy,hashlib,json
from arena_autonomy.follow_trace import MotionConfig,FollowConfig,replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.bypass_entry_trial import EntryConfig

class WideProbe(EntryConfig):
    @property
    def maximum_offset_m(self):return .35

A=Path(__file__).resolve().parent;trace=A.parent/'short_entry_transition_v1/run1/follow_decisions.jsonl'
count=mismatches=0;probes=[]
for line in trace.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        motion=MotionConfig(**d['motion_config'])
        f=BypassHandoffFollow(FollowConfig(**d['follow_config']),HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
    elif e['kind']=='reset':f.reset()
    elif e['kind']=='call':
        i=d['input'];t=i['reference_s'];original=d['output']
        if t>11.7 and original[2]['reason']=='handoff_entry':
            assert f.bypass.guide.path is None
            for maximum in (.27,.305,.31):
                cf=copy.deepcopy(f);guide=cf.bypass.guide
                options=asdict(guide.registered_entry);options['offsets_m']=(0.,-.16,.16,-.20,.20,-.24,.24,-maximum,maximum)
                guide.registered_entry=guide.entry=WideProbe(**options)
                out=freeze(replay_call(cf,i,motion));b=out[2].get('bypass') or {};ad=out[2].get('entry_admission') or {}
                probes.append(dict(maximum_offset_m=maximum,t=t,command=out[:2],reason=out[2]['reason'],selected=b.get('selected'),plan=b.get('corridor_plan'),admission=ad,candidates=[{k:r.get(k) for k in ('offset_m','transition_m','steering_rad','eligible','reason','command_clearance_m','preview_return_progress_m','preview_point_gap_m')} for r in b.get('candidates',[])]))
        actual=freeze(replay_call(f,i,motion));count+=1
        mismatches+=not equal_output(comparable_decision(actual),comparable_decision(original))
        if probes:break
assert mismatches==0 and len(probes)==3
out=dict(scope='동일 이력·단일 진입 판단 비교. 실제 이후 경로/조향/노면/통과는 미검증. 새 최대 편이는 고정 보호 여유가 아니라 후보 목표 범위이며 코드 기본값에 미연결.',source_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),matched_calls=count,mismatches=mismatches,probes=probes)
(A/'result.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8');print(json.dumps(dict(matched_calls=count,probes=[{k:v for k,v in r.items() if k in ('maximum_offset_m','t','command','reason','plan')} for r in probes]),ensure_ascii=False))

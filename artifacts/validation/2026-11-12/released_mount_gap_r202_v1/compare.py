"""r200 실패 궤적 고정 비교. 기존 인접 구조 제한을 그대로 사용한다."""
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
from arena_autonomy import released_body_gap as gap, current_path_recheck
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,restore_call,freeze,equal_output,comparable_decision
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
A=Path(__file__).resolve().parent
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
candidate=module('r202_candidate',A/'candidate.py')
route=module('r202_route',A.parent/'expiry_route_reference_r186_v1/candidate_v2.py');route.install()
path=A.parent/'released_gap_runtime_r200_v1/two_actor_run1/run1/follow_decisions.jsonl'
events=[json.loads(line) for line in path.open()];header=events[0]['data'];cfg=FollowConfig(**header['follow_config'])
old_check=gap.check
all_outputs={};mismatches=[];currents=[];proofs=Counter()
for enabled in (False,True):
    gap.check=candidate.check if enabled else old_check
    owner,motion=make_controller(header);out={}
    for event in events[1:]:
        data=event['data'];seq=event['seq']
        if event['kind']=='reset':
            if data.get('reason')=='decision_source_expired':route.reset_reference(owner)
            else:owner.reset()
        if event['kind']!='call':continue
        result=freeze(replay_call(owner,data['input'],motion));out[seq]=result
        if not enabled:
            if not equal_output(comparable_decision(result),comparable_decision(data['output'])):mismatches.append(seq)
        elif result[0]>0 and data['output'][0]<=0:
            proof=((result[2].get('normal') or {}).get('selector_released_body_gap') or {})
            proofs[(proof.get('attempts') or [{}])[-1].get('proof',{}).get('reason','none')]+=1
            item=data['input'];_,scan,history=restore_call(item,motion)
            scene=motion_scene(scan,history,item['reference_s'],timing_verified=item['timing_verified'],epoch=item['epoch'],rear_half_angle_deg=item['rear_half_angle_deg'])
            checked=current_path_recheck.rebase(result,item,item,scene,history,cfg)
            currents.append(dict(seq=seq,planned=result[:2],current=checked))
    all_outputs[enabled]=out
gap.check=old_check
counts=Counter();changes=[]
for seq,before in all_outputs[False].items():
    after=all_outputs[True][seq]
    if not equal_output(comparable_decision(before),comparable_decision(after)):
        changed=not equal_output(before[:2],after[:2])
        counts['detail_changes']+=1;counts['command_changes']+=changed
        counts['new_positive']+=before[0]<=0<after[0];counts['new_stop']+=before[0]>0>=after[0]
        counts['mode_changes']+=(before[2].get('normal') or {}).get('selector_mode')!=(after[2].get('normal') or {}).get('selector_mode')
        changes.append(dict(seq=seq,before=before[:2],after=after[:2]))
summary=dict(calls=len(all_outputs[False]),mismatches=mismatches,counts=dict(counts),proofs=dict(proofs),
    current_positive=sum(bool(c['current']) and c['current']['speed_mps']>0 for c in currents),
    current_reasons=dict(Counter(c['current']['reason'] if c['current'] else 'unsupported' for c in currents)),
    currents=currents,changes=changes,trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    scope='saved-input proposal; unchanged original risk limits; not new driving or default activation')
with (A/'comparison.json').open('x') as f:json.dump(summary,f,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ('currents','changes')}))
assert not mismatches

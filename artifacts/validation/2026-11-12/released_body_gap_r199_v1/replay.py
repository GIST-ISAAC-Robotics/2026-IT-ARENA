"""기본 OFF 전체 재현과 ON 변경, 같은 현재 입력의 공통 재검사를 대조한다."""
from collections import Counter
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy import current_path_recheck

A=Path(__file__).resolve().parent
TRACE=A.parent/'expiry_route_runtime_r187_v1/two_actor_run1/run1/follow_decisions.jsonl'
spec=importlib.util.spec_from_file_location('r199_route',A.parent/'expiry_route_reference_r186_v1/candidate_v2.py')
route=importlib.util.module_from_spec(spec);spec.loader.exec_module(route);route.install()
events=[json.loads(line) for line in TRACE.open()]
outputs={};mismatches=[];current_rows=[]
for enabled in (False,True):
    header=copy.deepcopy(events[0]['data'])
    header['follow_config'].update(guard_observation_policy='selected_r181',released_body_gap_trial=enabled)
    owner,motion=make_controller(header);cfg=FollowConfig(**header['follow_config'])
    outputs[enabled]={}
    for event in events[1:]:
        data=event['data']
        if event['kind']=='reset':
            if data.get('reason')=='decision_source_expired':route.reset_reference(owner)
            else:owner.reset()
        if event['kind']!='call':continue
        result=freeze(replay_call(owner,data['input'],motion));outputs[enabled][event['seq']]=result
        if not enabled and not equal_output(comparable_decision(result),comparable_decision(data['output'])):
            mismatches.append(event['seq'])
        candidate=(result[2].get('normal') or {}).get('selector_released_body_gap') or {}
        if enabled and candidate.get('accepted'):
            item=data['input'];_,scan,history=restore_call(item,motion)
            scene=motion_scene(scan,history,item['reference_s'],timing_verified=item['timing_verified'],
                epoch=item['epoch'],rear_half_angle_deg=item['rear_half_angle_deg'])
            checked=current_path_recheck.rebase(result,item,item,scene,history,cfg)
            current_rows.append(dict(seq=event['seq'],planned=result[:2],current=checked))
changes=[];counts=Counter()
for seq,before in outputs[False].items():
    after=outputs[True][seq]
    if not equal_output(comparable_decision(before),comparable_decision(after)):
        command_change=not equal_output(before[:2],after[:2])
        counts['detail_changes']+=1;counts['command_changes']+=command_change
        counts['new_stop']+=before[0]>0 and after[0]<=0
        counts['new_positive']+=before[0]<=0 and after[0]>0
        counts['selector_mode_changes']+=(before[2].get('normal') or {}).get('selector_mode')!=(after[2].get('normal') or {}).get('selector_mode')
        changes.append(dict(seq=seq,before=before[:2],after=after[:2],
            reason=(after[2].get('normal') or {}).get('selector_reason')))
summary=dict(calls=len(outputs[False]),off_mismatches=mismatches,counts=dict(counts),changes=changes,
    current_cases=len(current_rows),current_positive=sum(bool(r['current']) and r['current']['speed_mps']>0 for r in current_rows),
    current_reasons=dict(Counter(None if r['current'] is None else r['current']['reason'] for r in current_rows)),
    current_rows=current_rows,input_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
    scope='fixed_input_replay_and_same_current_snapshot_only_not_new_trajectory_or_live_deadline')
with (A/'replay.json').open('x') as stream:json.dump(summary,stream,indent=2);stream.write('\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ('changes','current_rows')}))
assert not mismatches and not counts['new_stop'] and not counts['selector_mode_changes']

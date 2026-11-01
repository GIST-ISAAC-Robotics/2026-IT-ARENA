"""기존 기록과 새 후보를 같은 센서 입력에서 비교. 실제 궤적 변화는 별도."""
from dataclasses import replace
from hashlib import sha256
from collections import Counter
from pathlib import Path
import json,sys
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig


def run(folder):
    report=json.loads((folder/'report.json').read_bytes());green=report['case_window']['green_sim_s']
    assert report['run_valid'] and report['source_unchanged']
    counts=Counter();changed=[];selected=Counter();digest=sha256();footer=None
    with (folder/'follow_decisions.jsonl').open('rb') as f:
        for seq,line in enumerate(f):
            digest.update(line);e=json.loads(line);d=e['data'];assert seq==e['seq'] and footer is None
            if e['kind']=='header':
                config=FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
                execution=HandoffExecutionConfig.from_mapping(d['execution_config'])
                original=BypassHandoffFollow(config,execution,allow_simulation_trial=True)
                trial=BypassHandoffFollow(replace(config,released_rear_sweep_trial=True),execution,allow_simulation_trial=True)
            elif e['kind']=='footer':footer=d
            elif e['kind']=='reset':original.reset();trial.reset()
            elif e['kind']=='call':
                inp=d['input'];t=inp['reference_s']-green
                if t>40:continue
                old=d['output'];actual=replay_call(original,inp,motion)
                assert abs(old[0]-actual[0])<1e-8 and abs(old[1]-actual[1])<1e-8 and old[2]['handoff_state']==actual[2]['handoff_state'],seq
                counts['baseline_reproduced']+=1
                new=replay_call(trial,inp,motion);detail=new[2].get('normal',{})
                accepted=detail.get('selector_released_rear_sweep_risk_accepted',False)
                selected[str(accepted)]+=1
                if abs(old[0]-new[0])>1e-8 or abs(old[1]-new[1])>1e-8 or old[2]['handoff_state']!=new[2]['handoff_state']:
                    counts['changed']+=1;counts['new_zero']+=int(old[0]>.001>=new[0])
                    counts['zero_released']+=int(new[0]>.001>=old[0])
                    changed.append(dict(t=t,old_v=old[0],new_v=new[0],old_a=old[1],new_a=new[1],
                        mode=new[2]['handoff_state'],reason=detail.get('reason'),selector=detail.get('selector_reason'),
                        rear=detail.get('selector_released_rear_sweep'),region=detail.get('selector_region')))
    assert digest.hexdigest()==report['follow_decisions']['sha256']
    assert footer['written']==seq and not any(footer[k] for k in ('failed','dropped','capture_errors','pending'))
    return dict(run=str(folder),counts=dict(counts),selected=dict(selected),changed=changed,
        scope='첫40초 이하 고정 센서 입력. 실제 진행·상대 간격·새 조향 궤적은 아직 검증하지 않음.')


if __name__=='__main__':
    values=[run(Path(n)) for n in sys.argv[2:]]
    with Path(sys.argv[1]).open('x',encoding='utf-8') as f:json.dump(values,f,ensure_ascii=False,indent=2)
    print(json.dumps([dict(run=v['run'],counts=v['counts'],selected=v['selected']) for v in values]))

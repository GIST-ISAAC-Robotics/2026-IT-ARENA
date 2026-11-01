"""기록된 입력의 초기 30초에서 원래 출력을 재현하고 양보 갱신 후보만 비교한다."""
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
    digest=sha256();counts=Counter();changed=[];epochs=[];footer=None
    with (folder/'follow_decisions.jsonl').open('rb') as f:
        for seq,line in enumerate(f):
            digest.update(line);e=json.loads(line);d=e['data']
            assert e['seq']==seq and footer is None
            if e['kind']=='header':
                cfg=FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
                ex=HandoffExecutionConfig.from_mapping(d['execution_config'])
                original=BypassHandoffFollow(cfg,ex,allow_simulation_trial=True)
                trial=BypassHandoffFollow(cfg,replace(ex,relative_yield_policy='observed_progress_yield_v3'),allow_simulation_trial=True)
            elif e['kind']=='footer':footer=d
            elif e['kind']=='reset':original.reset();trial.reset()
            elif e['kind']=='call':
                inp=d['input'];t=inp['reference_s']-green
                if t>30:continue
                v,a,q=d['output'];v0,a0,q0=replay_call(original,inp,motion)
                assert abs(v-v0)<1e-8 and abs(a-a0)<1e-8 and q['handoff_state']==q0['handoff_state'],seq
                counts['baseline_reproduced']+=1
                vn,an,qn=replay_call(trial,inp,motion)
                yd=(qn.get('relative_yield') or {}).get('current_observer') or {}
                if yd.get('progress_budget_renewed'):epochs.append(dict(t=t,detail=yd))
                if abs(v-vn)>1e-7 or abs(a-an)>1e-8 or q['handoff_state']!=qn['handoff_state']:
                    counts['changed']+=1;counts['new_zero']+=int(v>.001>=vn)
                    counts['lower']+=int(vn<v-1e-7);counts['higher']+=int(vn>v+1e-7)
                    changed.append(dict(t=t,old=v,new=vn,angle_old=a,angle_new=an,mode=qn['handoff_state'],
                        observer=yd,old_reason=(q.get('bypass') or {}).get('reason'),new_reason=(qn.get('bypass') or {}).get('reason')))
    assert footer and footer['written']==seq and not any(footer[x] for x in ('failed','dropped','capture_errors','pending'))
    assert digest.hexdigest()==report['follow_decisions']['sha256']
    return dict(run=str(folder),counts=dict(counts),renewals=epochs,changed=changed,
        scope='30초 이하의 고정 입력 비교. 정답 자세 미입력. 새 감속이 실제로 만든 상대 간격·복귀는 새 실행에서 확인 필요.')

if __name__=='__main__':
    out=Path(sys.argv[1])
    data=[run(Path(x)) for x in sys.argv[2:]]
    with out.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
    print(json.dumps([dict(run=x['run'],counts=x['counts'],renewals=len(x['renewals'])) for x in data]))

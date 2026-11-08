"""HOLD가 누락한 실제 복귀 단계를 기존 G1 인계에 전달하는 후보만 저장 입력에서 비교. 실제 출력/채택 없음."""
import json
from pathlib import Path
from unittest.mock import patch
import numpy as np
from arena_autonomy import bypass_entry_trial as entry
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig

HERE=Path(__file__).resolve().parent
events=[json.loads(l) for l in (HERE/'persistent_block/run1/follow_decisions.jsonl').read_text().splitlines()]
h=events[0]['data'];mc=MotionConfig(**h['motion_config'])
ctl=BypassHandoffFollow(FollowConfig(**h['follow_config']),HandoffExecutionConfig.from_mapping(h['execution_config']),allow_simulation_trial=True)
original_fit=entry.EntryGuide._outer_wall_fit
original_covers=entry.EntryGuide._wall_covers_guide
original_rejoin=BypassHandoffFollow._observed_rejoin
original_swept=entry.swept_limit
context={}; changes=[]; hits=[]; proof=[]

def fit(self,scene,remembered):
    f,w=original_fit(self,scene,remembered)
    context.update(scene=scene,fit=f,wall=w,phase=self.phase)
    return f,w

def rejoin(self,normal,candidate):
    if (self.execution.bypass.rear_wall_witness and
            candidate[2].get('entry_phase') is None):
        candidate=(*candidate[:2],dict(candidate[2],entry_phase=self.bypass.guide.phase))
        proof.append(dict(t=context.get('scene').t_ref_s if context.get('scene') else None,
                          phase=self.bypass.guide.phase,reason=candidate[2].get('reason')))
    return original_rejoin(self,normal,candidate)


def swept(points,steering,*args,**kwargs):
    result=original_swept(points,steering,*args,**kwargs)
    scene=context.get('scene')
    if result==0 and scene is not None and any(abs(scene.t_ref_s-t)<.001 for t in [13.5,16.,20.]):
        p=np.asarray(points);pad=np.broadcast_to(kwargs.get('margin',.025),(len(p),))
        # 실제 검사 시작 자세의 간섭. 미래 곡률 부분은 별도라 0 원인을 전부 설명하지 않는다.
        mask=(abs(p[:,0])<=kwargs.get('length',.20)/2+pad)&(abs(p[:,1])<=kwargs.get('width',.15)/2+pad)
        ids=[r.source_index for r in scene.rays if r.status=='return']
        rows=[dict(ray=ids[i],xy=p[i].tolist(),padding=float(pad[i])) for i in np.flatnonzero(mask)]
        hits.append(dict(t=scene.t_ref_s,steering=steering,points=rows,
                         wall=context.get('wall',{}).get('source_ray_indices',[])))
    return result

with patch.object(entry.EntryGuide,'_outer_wall_fit',fit),patch.object(BypassHandoffFollow,'_observed_rejoin',rejoin),patch.object(entry,'swept_limit',swept):
    for event in events[1:]:
        if event['kind']=='reset':ctl.reset()
        elif event['kind']=='call':
            d=event['data'];context.clear();out=freeze(replay_call(ctl,d['input'],mc));old=d['output']
            if not equal_output(out[:2],old[:2]):
                changes.append(dict(t=d['input']['reference_s'],before=old[:2],after=out[:2],
                    reason=out[2].get('bypass',{}).get('reason'),state=out[2].get('handoff_state')))
result=dict(scope='fixed feedback only; no actual return proof',changed=len(changes),
    positive_from_zero=sum(r['before'][0]==0 and r['after'][0]>0 for r in changes),
    changes=changes,coverage_evidence=proof,initial_overlap=hits)
(HERE/'phase_shadow.json').write_text(json.dumps(freeze(result),ensure_ascii=False,indent=1)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k not in ('changes','coverage_evidence','initial_overlap')},ensure_ascii=False))

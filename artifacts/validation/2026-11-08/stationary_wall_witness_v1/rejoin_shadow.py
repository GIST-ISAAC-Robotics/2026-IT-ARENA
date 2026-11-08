"""앞쪽 현재 벽을 쓰는 복귀 후보만 저장 입력에서 비교. 실제 출력/채택 없음."""
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
original_swept=entry.swept_limit
context={}; changes=[]; hits=[]; proof=[]

def fit(self,scene,remembered):
    f,w=original_fit(self,scene,remembered)
    context.update(scene=scene,fit=f,wall=w,phase=self.phase)
    return f,w

def covers(self,f,w):
    if original_covers(self,f,w):return True
    # 이미 양의 관측으로 시작한 복귀 경로의 전방 목표만 현재 관측에서 고른다.
    # 벽 길이는 경로 기준이며 실제 전방/측방 이동은 기존 점유·관측 검사 대상이다.
    accepted=(self.rear_wall_witness_enabled and self.phase=='rejoin'
        and w.get('reason')=='current_connected_straight_wall_fit'
        and w.get('actual_return_count',0)>=12
        and f[2]<=self.config.length_m/2 and f[3]>=.65)
    if accepted:
        proof.append(dict(t=context['scene'].t_ref_s,fit=f,wall_count=w['actual_return_count']))
    return bool(accepted)

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

with patch.object(entry.EntryGuide,'_outer_wall_fit',fit),patch.object(entry.EntryGuide,'_wall_covers_guide',covers),patch.object(entry,'swept_limit',swept):
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
(HERE/'rejoin_shadow.json').write_text(json.dumps(freeze(result),ensure_ascii=False,indent=1)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k not in ('changes','coverage_evidence','initial_overlap')},ensure_ascii=False))

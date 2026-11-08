"""r16 고정 입력의 재현과 현재 벽 범위/정체 원인 수집. 제어 출력은 바꾸지 않는다."""
from collections import Counter
import json
from pathlib import Path
from unittest.mock import patch
from arena_autonomy.bypass_entry_trial import EntryGuide
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig

HERE=Path(__file__).resolve().parent
events=[json.loads(l) for l in (HERE/'persistent_block/run1/follow_decisions.jsonl').read_text().splitlines()]
h=events[0]['data']; mc=MotionConfig(**h['motion_config'])
ctl=BypassHandoffFollow(FollowConfig(**h['follow_config']),HandoffExecutionConfig.from_mapping(h['execution_config']),allow_simulation_trial=True)
original=EntryGuide._outer_wall_fit
evidence={};rows=[]
def fit(self,scene,remembered):
    f,w=original(self,scene,remembered)
    evidence.clear();evidence.update(phase=self.phase,fit=f,side=self.side,
        wall={k:w.get(k) for k in ['reason','actual_return_count','rear_geometry_witness']},
        band=self._rejoin_band(scene,f) if f is not None else None)
    return f,w
with patch.object(EntryGuide,'_outer_wall_fit',fit):
    for event in events[1:]:
        if event['kind']=='reset':ctl.reset()
        elif event['kind']=='call':
            d=event['data'];evidence.clear()
            out=freeze(replay_call(ctl,d['input'],mc));old=d['output']
            assert equal_output(comparable_decision(out),comparable_decision(old))
            b=out[2].get('bypass',{})
            rows.append(dict(t=d['input']['reference_s'],measured_speed=d['input']['speed_mps'],
                output=out[:2],reason=b.get('reason'),evidence=freeze(evidence),
                candidates=[{k:r.get(k) for k in ['reason','eligible','speed_mps','steering_rad','command_speed_mps','motion_pause']} for r in b.get('candidates',[])],
                selected=b.get('selected'),motion_pause=b.get('motion_pause')))
(HERE/'diagnosis.json').write_text(json.dumps(rows,ensure_ascii=False,indent=1)+'\n',encoding='utf8')
summary=dict(calls=len(rows),all_equal=True,zero_reasons=dict(Counter(str(r['reason']) for r in rows if r['output'][0]==0)),
    samples=[min(rows,key=lambda r:abs(r['t']-t)) for t in [13.5,16.,20.,23.,26.5]])
(HERE/'diagnosis_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in summary.items() if k!='samples'},ensure_ascii=False))

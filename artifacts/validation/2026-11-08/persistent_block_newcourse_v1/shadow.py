"""저장 입력에서만 이동차의 기존 후방 벽 근거를 정지차 경로에 대조한다."""
from collections import Counter
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
from unittest.mock import patch
import numpy as np

from arena_autonomy.bypass_entry_follow import BypassEntryFollow
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.bypass_handoff_corridor_trial import ForecastGuide
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output
from arena_autonomy.known_structure import classify, parse_sectors
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.moving_pass import rear_wall_witness

HERE=Path(__file__).resolve().parent
SOURCE=HERE/'persistent_block/run1/follow_decisions.jsonl'
events=[json.loads(line) for line in SOURCE.read_text().splitlines()]
header=events[0]['data']
original_command=BypassEntryFollow.command
original_fit=ForecastGuide._outer_wall_fit


def replay(enabled):
    ctl=BypassHandoffFollow(FollowConfig(**header['follow_config']),
        HandoffExecutionConfig.from_mapping(header['execution_config']),allow_simulation_trial=True)
    motion=MotionConfig(**header['motion_config'])
    evidence=[]; rows=[]
    def command(self,*args,raw_scan=None,**kwargs):
        self.guide.raw_scan=raw_scan
        return original_command(self,*args,raw_scan=raw_scan,**kwargs)
    def fit(self,scene,remembered):
        initial,wall=original_fit(self,scene,remembered)
        candidate=initial; new_wall=wall; scan=getattr(self,'raw_scan',None)
        if (initial is not None and initial[2]>-self.config.wheelbase_m/2 and scan is not None
                and self.config.known_structure_policy=='structure_aware_v1'):
            sectors=parse_sectors(self.config.known_structure_sectors_json)
            _,_,classified=classify(scan,sectors)
            explained={frozenset(q['rays'].tolist()) for q in classified if q['explained']}
            candidate,new_wall=rear_wall_witness(scene,self.config,self.side,initial,wall,explained)
            if len(evidence)<12 or candidate!=initial:
                evidence.append(dict(t=scene.t_ref_s,before=initial,after=candidate,
                    reason=wall.get('reason'),witness=new_wall.get('rear_geometry_witness'),side=self.side))
        return (candidate,new_wall) if enabled else (initial,wall)
    with ExitStack() as stack:
        stack.enter_context(patch.object(BypassEntryFollow,'command',command))
        stack.enter_context(patch.object(ForecastGuide,'_outer_wall_fit',fit))
        for event in events[1:]:
            if event['kind']=='reset':ctl.reset()
            elif event['kind']=='call':
                data=event['data']; out=freeze(replay_call(ctl,data['input'],motion))
                old=data['output']
                rows.append(dict(t=data['input']['reference_s'],before=old[:2],after=out[:2],
                    same=equal_output(comparable_decision(out),comparable_decision(old)),
                    numeric_same=bool(np.allclose(out[:2],old[:2],rtol=0,atol=1e-9)),
                    reason=out[2].get('reason'),bypass_reason=out[2].get('bypass',{}).get('reason')))
    return rows,evidence


baseline,proof=replay(False)
assert all(r['same'] for r in baseline),'baseline differs; do not interpret candidate'
candidate,proof=replay(True)
changed=[r for r in candidate if not r['numeric_same']]
result=dict(scope='fixed recorded feedback only, not actual departure or adoption',
    source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),baseline_calls=len(baseline),
    baseline_all_equal=True,changed_numeric=len(changed),first_changes=changed[:4],
    positive_from_zero=sum(r['before'][0]==0 and r['after'][0]>0 for r in changed),
    changed_reasons=dict(Counter(r['bypass_reason'] for r in changed)),
    witness_count=sum(q['after']!=q['before'] for q in proof),wall_evidence=proof[:12])
(HERE/'shadow_result.json').write_text(json.dumps(freeze(result),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
(HERE/'shadow_changes.json').write_text(json.dumps(changed,ensure_ascii=False,indent=1)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k not in ('wall_evidence','first_changes')},ensure_ascii=False))

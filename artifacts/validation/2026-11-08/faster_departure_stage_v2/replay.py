"""실제 경로 생성 및 상태기를 유지하고, 빠른 추월 경로 판단에만 후보를 적용."""
from collections import Counter
from pathlib import Path
from hashlib import sha256
import json
from arena_autonomy.follow_trace import FollowConfig, MotionConfig, replay_call, freeze, equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.moving_pass import MovingPassGuide
from pass_side_departure import compare_pass_side_departure

HERE=Path(__file__).resolve().parent
TRACE=HERE.parent/'moving_pass_wall_mask_v1/straight_moving_lead/run1/follow_decisions.jsonl'
original=MovingPassGuide.evaluate


def evaluate(self,scene,history,speed,previous,dt):
    tracked=self.path is not None
    result=original(self,scene,history,speed,previous,dt)
    if tracked and self.entry.profile=='faster_v1':
        result=compare_pass_side_departure(result,scene,self.config,speed,previous,dt,
            target_observation=self.tracker.detail,raw_scan=self.raw_scan,sectors=self.sectors,
            cap=self.entry.speed_cap_mps)
    return result


def main():
    counts=Counter(); reasons=Counter(); rows=[]
    MovingPassGuide.evaluate=evaluate
    try:
        for l in TRACE.open(encoding='utf8'):
            e=json.loads(l);d=e['data']
            if e['kind']=='header':
                mc=MotionConfig(**d['motion_config'])
                engine=BypassHandoffFollow(FollowConfig(**d['follow_config']),
                    HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
            if e['kind']=='reset':engine.reset()
            if e['kind']!='call':continue
            old=d['output'];new=freeze(replay_call(engine,d['input'],mc)); x=d['input']
            proof=new[2].get('moving_pass',{}).get('entry',{}).get('pass_side_departure')
            counts['calls']+=1;counts['old_zero']+=old[0]==0;counts['new_zero']+=new[0]==0
            counts['new_hold']+=old[0]>0 and new[0]==0
            counts['released']+=old[0]==0 and new[0]>0
            changed=not equal_output(old[:2],new[:2])
            counts['changed']+=changed
            if proof:reasons[proof.get('decision_reason','accepted')]+=1
            rows.append(dict(t=x['reference_s'],before=old[:2],after=new[:2],
                accepted=bool(proof and proof['accepted']),changed=changed,reason=new[2].get('reason')))
    finally:
        MovingPassGuide.evaluate=original
    (HERE/'rows.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows),encoding='utf8')
    evidence=dict(counts=counts,reasons=reasons,first=next((r for r in rows if r['changed']),None),
        trace_sha256=sha256(TRACE.read_bytes()).hexdigest(),helper_sha256=sha256((HERE/'pass_side_departure.py').read_bytes()).hexdigest(),
        rows_sha256=sha256((HERE/'rows.jsonl').read_bytes()).hexdigest(),scope='fixed input, not a new trajectory')
    (HERE/'summary.json').write_text(json.dumps(evidence,indent=2)+'\n',encoding='utf8')
    print(json.dumps(evidence),flush=True)


if __name__=='__main__':main()

"""run1 기록을 읽어 판단 시계열과 독립 보호 함수 대조를 만든다. 새 주행 없음.

독립 보호 대조는 저장 입력·저장 명령을 현재 보호 함수에 넣은 그림자 계산이다. 당시 ROS 수신 순서를
재현하지 않는다. 실제 보호 출력은 run1 기록에서 따로 읽는다.
"""
from collections import Counter
import json
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts'),
    str(ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1')]
from safety_shadow import ShadowSafety
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig


def main(run=BASE/'run1'):
    events=[json.loads(line) for line in (run/'g2_decisions.jsonl').open(encoding='utf-8') if line.strip()]
    header=events[0]['data']
    config=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
    guard=ShadowSafety(None,config,30.)
    rows=[];reduced=0;reasons=Counter();safety=Counter()
    for event in events:
        if event['kind']!='call':
            continue
        d=event['data'];inp=d['input'];out=d['output']
        target,steering,detail=out[0],out[1],out[2]
        safe,status=guard.step(inp,motion,(target,steering))
        reduced+=safe[0]<target-1e-12
        safety[status['reason']]+=1;reasons[detail.get('reason')]+=1
        plan=detail.get('corridor_plan') or {}
        side=detail.get('corridor',{})
        use=next(iter(side.values()),{}) if side else {}
        state=use.get('state',{})
        rows.append(dict(t=inp['reference_s'],speed=inp['speed_mps'],target=target,steering=steering,safe=safe[0],
            reason=detail.get('reason'),phase=detail.get('entry_phase'),selected=detail.get('selected'),
            stable=state.get('stable'),approach=state.get('approach_m'),inner=(use.get('surface') or {}).get('normal_m'),
            padding=use.get('maximum_inner_padding_m'),onset=use.get('onset_rays'),aligned=use.get('inner_rays_aligned'),
            wall=use.get('wall_rays_aligned'),plan_chosen=plan.get('chosen'),guard=status['reason']))
    positive=[r for r in rows if r['target']>0]
    summary=dict(calls=len(rows),positive_commands=len(positive),shadow_guard_reduced=reduced,shadow_guard_reasons=dict(safety),
        reasons=dict(reasons),first_bypass=next((r for r in rows if r['selected'] and r['selected']['offset_m']),None),
        plan_chosen=next((r['plan_chosen'] for r in rows if r['plan_chosen']),None),
        phases={p:next(r['t'] for r in rows if r['phase']==p) for p in {r['phase'] for r in rows if r['phase']}},
        unstable_after_entry=sum(r['stable'] is False for r in rows if r['phase']),
        scope='shadow function comparison on recorded inputs; not a ROS transport replay')
    (BASE/'run1_decision_series.json').write_text(json.dumps(rows)+'\n',encoding='utf-8')
    (BASE/'run1_analysis.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False)[:1500])


if __name__=='__main__':
    main()

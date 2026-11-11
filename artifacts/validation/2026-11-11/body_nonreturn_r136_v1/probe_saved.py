"""같은 저장 입력에서 속도·조향·정지 변화를 비교한다. 실제 주행 재현은 아니다."""
from collections import Counter
import hashlib
import json
from pathlib import Path

from candidate import install,uninstall
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze

A=Path(__file__).resolve().parent
TRACE=A.parent/'parallel_rebase_r133_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    digest=hashlib.sha256(TRACE.read_bytes()).hexdigest()
    events=[json.loads(line) for line in TRACE.open(encoding='utf8')]
    install();owner,motion=make_controller(events[0]['data'])
    rows=[];checks=[]
    for row in events[1:]:
        if row['kind']=='reset': owner.reset();continue
        if row['kind']!='call':continue
        old=row['data']['output'];new=freeze(replay_call(owner,row['data']['input'],motion))
        oldn=old[2].get('normal') or {};newn=new[2].get('normal') or {}
        detail=(newn.get('selector_guard_candidate') or {}).get('same_path_gap') or newn.get('same_path_gap') or {}
        rows.append(dict(seq=row['seq'],reference_s=row['data']['input']['reference_s'],
            before=old[:2],after=new[:2],before_reason=oldn.get('reason'),after_reason=newn.get('reason'),
            selector_before=oldn.get('selector_reason'),selector_after=newn.get('selector_reason'),
            candidate=detail,region=(newn.get('selector_guard_candidate') or {}).get('region'),
            sides_before=oldn.get('selector_sides'),sides_after=newn.get('selector_sides')))
    uninstall()
    report=dict(calls=len(rows),zero_to_positive=sum(r['before'][0]<=0<r['after'][0] for r in rows),
        positive_to_zero=sum(r['after'][0]<=0<r['before'][0] for r in rows),
        commands_changed=sum(r['before']!=r['after'] for r in rows),
        structure_accepted=sum(r['candidate'].get('policy')=='guard_low_speed_body_nonreturn_r136' and r['candidate'].get('accepted') is True for r in rows),
        structure_rejections=dict(Counter(r['candidate']['body_nonreturn_recheck']['reason'] for r in rows if 'body_nonreturn_recheck' in r['candidate'])),
        trace_sha256=digest,trace_unchanged=hashlib.sha256(TRACE.read_bytes()).hexdigest()==digest,
        scope='stored inputs only; closed-loop motion and command delivery not reproduced')
    (A/'comparison.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    (A/'comparison_details.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows),encoding='utf8')
    print(json.dumps(report))


if __name__=='__main__':main()

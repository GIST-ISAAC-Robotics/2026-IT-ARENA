"""선별 GUARD 거부에서 기존 저속 후보가 같은 전체 검사를 통과하는지만 비교."""
import copy,hashlib,json,sys
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy import pass_release_trial as pr
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import freeze,replay_call,equal_output,comparable_decision


def main():
    source=A.parent/'async_corner_r105_v1/two_actor_run1/run1/follow_decisions.jsonl'
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    rows=[json.loads(line) for line in source.read_text('utf8').splitlines()]
    follow,motion=make_controller(rows[0]['data'])
    original=pr.PassReleaseSelector._guard_curve
    active={};attempts=[];mismatch=[]
    chosen={636,640,644,645,650,700,975}
    def probe(self,*args,**kwargs):
        result=original(self,*args,**kwargs)
        if active['seq'] in chosen and result is not None:
            cases=[]
            for cap in (.25,.15):
                owner,arguments,keywords=copy.deepcopy((self,args,kwargs))
                candidate=owner._guard_curve_at_cap(*arguments,**keywords,target_cap=cap)
                if candidate is None:
                    cases.append(dict(cap_mps=cap,not_applicable=True));continue
                c=candidate[2].get('selector_guard_candidate',{})
                cases.append(dict(cap_mps=cap,output=candidate[:2],reason=candidate[2].get('selector_reason'),
                    prediction_speed_mps=c.get('prediction_speed_mps'),accepted=c.get('accepted'),
                    guarded_sides=c.get('guarded_sides'),region=c.get('region'),approach=c.get('current_approach')))
            attempts.append(dict(seq=active['seq'],reference_s=args[0].t_ref_s,
                actual_output=result[:2],actual_reason=result[2].get('selector_reason'),candidates=cases))
        return result
    pr.PassReleaseSelector._guard_curve=probe
    try:
        for row in rows[1:]:
            if row['kind']=='reset':follow.reset()
            elif row['kind']=='call':
                active['seq']=row['seq']
                actual=freeze(replay_call(follow,row['data']['input'],motion))
                if not equal_output(comparable_decision(actual),comparable_decision(row['data']['output'])):
                    mismatch.append(row['seq'])
    finally:pr.PassReleaseSelector._guard_curve=original
    assert hashlib.sha256(source.read_bytes()).hexdigest()==digest
    with (A/'guard_caps.json').open('x',encoding='utf8') as output:
        json.dump(freeze(dict(source=source.relative_to(ROOT).as_posix(),sha256=digest,mismatches=mismatch,attempts=attempts)),output,indent=2)
    print(json.dumps(dict(mismatches=mismatch,attempts=[dict(seq=a['seq'],actual=a['actual_output'],candidates=[{k:v for k,v in c.items() if k in ('cap_mps','accepted','reason','prediction_speed_mps')} for c in a['candidates']]) for a in attempts])))


if __name__=='__main__':main()

"""저장 지연 입력에서 전체 reset과 계산 전 상태 복원을 비교. 실행 연결 없음."""
import collections
import copy
import hashlib
import json
from pathlib import Path
import statistics
import time

from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import comparable_decision, equal_output, freeze, replay_call

A=Path(__file__).resolve().parent
SOURCE=A.parent/'decision_delay_r110_v1/two_actor_run1/run1/follow_decisions.jsonl'
EXPIRY={'decision_source_expired','decision_command_expired'}


def main():
    digest=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    rows=[json.loads(line) for line in SOURCE.open(encoding='utf8')]
    calls=[r for r in rows if r['kind']=='call']; results={}; costs=[]
    for mode in ('baseline','rollback','retain_diagnostic_only'):
        controller,motion=make_controller(rows[0]['data']);outputs=[]; restored=0
        for row in rows[1:]:
            kind,data=row['kind'],row['data']
            if kind=='reset':
                if mode=='baseline' or data.get('reason') not in EXPIRY:
                    controller.reset()
            elif kind=='call':
                if mode=='rollback':
                    began=time.perf_counter();checkpoint=copy.deepcopy(controller)
                    costs.append((time.perf_counter()-began)*1000.)
                outputs.append(freeze(replay_call(controller,data['input'],motion)))
                delivery=data.get('delivery',{})
                if mode=='rollback' and delivery.get('kind')=='discarded' and delivery.get('reason') in EXPIRY:
                    controller=checkpoint;restored+=1
        results[mode]=dict(outputs=outputs,restored=restored)
    baseline=results['baseline']['outputs'];report={}
    for mode,result in results.items():
        out=result.pop('outputs'); changes=[];reasons=collections.Counter()
        accepted_zero=accepted_positive=discarded=0
        for row,before,after in zip(calls,baseline,out):
            accepted=row['data'].get('delivery',{}).get('kind')=='candidate'
            if accepted:
                accepted_zero+=after[0]==0;accepted_positive+=after[0]>0
                normal=after[2].get('normal',{})
                if after[0]==0:reasons[normal.get('reason',after[2].get('reason'))]+=1
                if not equal_output(before[:2],after[:2]):
                    changes.append(dict(seq=row['seq'],reference_s=row['data']['input']['reference_s'],
                        before=before[:2],after=after[:2],reason=normal.get('reason'),
                        safety=normal.get('partial_observation',{}).get('reason')))
            else:discarded+=1
        report[mode]={**result,'accepted_zero':accepted_zero,'accepted_positive':accepted_positive,
            'discarded_commands_still_not_applied':discarded,'zero_reasons':dict(reasons),'changes':changes}
    costs.sort()
    report.update(source_sha256=digest,calls=len(calls),
        baseline_mismatches=sum(not equal_output(comparable_decision(o),comparable_decision(r['data']['output'])) for o,r in zip(baseline,calls)),
        deepcopy_wall_ms=dict(p50=statistics.median(costs),p95=costs[int(.95*len(costs))],maximum=max(costs)),
        scope='same saved sensors and delivery schedule; counterfactual only, no closed-loop/contact improvement proof; retain is diagnostic, not adopted')
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest()==digest
    with (A/'comparison.json').open('x',encoding='utf8') as stream:json.dump(report,stream,indent=2)
    print(json.dumps({k:({kk:vv for kk,vv in v.items() if kk!='changes'} if isinstance(v,dict) else v) for k,v in report.items()}))


if __name__=='__main__':main()

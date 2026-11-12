"""실행 구조의 시간 예산만 비교한다. 제어 출력/주행 성공을 합성하지 않는다."""
import json
import math
from pathlib import Path
import numpy as np

A=Path(__file__).resolve().parent
ROOT=A.parents[3]

def source_at(t):
    return math.floor((t+1e-9)/.1)*.1

def simulate(costs, *, overlap, fresh):
    start=0.;last_scan=-1.;last_start=-1.;last_publish=-1.;current_free=0.
    accepted=[];reasons={};rows=[]
    for plan_ms,current_ms in costs:
        start=max(start,last_start+.1)
        scan=source_at(start)
        if scan<=last_scan+1e-9 or (fresh and start-scan>.025+1e-9):
            start=max(scan+.1,last_scan+.1)
            scan=source_at(start)
        finish_plan=start+plan_ms/1000
        current_start=max(finish_plan,current_free)
        finish=current_start+current_ms/1000
        current_free=finish
        publication=max(finish,last_publish+.1)
        reason=None
        if publication-(scan-.1)>=.3-1e-9:reason='original_source_age'
        elif publication-start>=.2-1e-9:reason='plan_command_age'
        elif publication-current_start>=.2-1e-9:reason='current_command_age'
        if reason is None:
            accepted.append((publication,current_start))
            last_publish=publication
        else:reasons[reason]=reasons.get(reason,0)+1
        rows.append(dict(start=start,scan_complete=scan,finish_plan=finish_plan,
                         current_start=current_start,publication=publication,discard=reason))
        last_start,last_scan=start,scan
        start=finish_plan if overlap else publication
    gaps=[b[0]-a[0] for a,b in zip(accepted,accepted[1:])]
    # 이전 현재 재검사의 기준 시각부터 다음 발행까지. 연속 0출력은 모형화하지 않는다.
    ages=[b[0]-a[1] for a,b in zip(accepted,accepted[1:])]
    return dict(submitted=len(rows),accepted_budget_only=len(accepted),discard_reasons=reasons,
                accepted_hz=(len(accepted)-1)/(accepted[-1][0]-accepted[0][0]) if len(accepted)>1 else 0,
                publication_gap_p95_ms=float(np.percentile(gaps,95)*1000),
                previous_command_age_before_next_p95_ms=float(np.percentile(ages,95)*1000),
                expired_intervals=sum(x>=.2-1e-9 for x in ages),
                no_steering_state_validation=True,rows=rows)

def main():
    measured=json.loads((ROOT/'artifacts/validation/2026-11-12/projection_kernel_r257_v1/jetson/baseline/durations.json').read_text())
    sets={'fixed_147_34':[(147.,34.)]*100,
          'fixed_100_34':[(100.,34.)]*100,
          'fixed_95_25':[(95.,25.)]*100,
          'jetson_pure_saved_costs':[(r['plan_ms'],r['current_ms'] or 0.) for r in measured]}
    reports=[]
    for label,costs in sets.items():
        for overlap,fresh in [(False,True),(True,True),(True,False)]:
            result=simulate(costs,overlap=overlap,fresh=fresh)
            reports.append(dict(costs=label,overlap=overlap,fresh_25ms=fresh,**result))
    (A/'result.json').write_text(json.dumps(reports,indent=2)+'\n')
    print(json.dumps([{k:v for k,v in r.items() if k!='rows'} for r in reports],indent=2))

if __name__=='__main__':main()

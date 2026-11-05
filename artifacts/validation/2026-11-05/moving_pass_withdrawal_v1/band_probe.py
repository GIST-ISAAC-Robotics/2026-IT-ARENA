"""포기 대상은 충분히 앞인데 복귀 후보가 나오지 않는 구간의 현재 띠 검사."""
import sys
from prepare import HERE,save,read
from assess import rows
from arena_autonomy.follow_trace import replay_events
from arena_autonomy.moving_pass import MovingPassGuide

if __name__=='__main__':
    run=HERE/'baseline_r12/run1';green=read(run/'report.json')['case_window']['green_sim_s']
    captured=[];last=None;original=MovingPassGuide._rejoin_band
    def probe(self,scene,fit):
        global last
        band=original(self,scene,fit)
        if last!=scene.source_first_s:
            captured.append(dict(t=round(scene.t_ref_s-green,3),fit=fit,band=band))
            last=scene.source_first_s
        return band
    MovingPassGuide._rejoin_band=probe
    try:result=replay_events(rows(run/'follow_decisions.jsonl'))
    finally:MovingPassGuide._rejoin_band=original
    save(HERE/(sys.argv[1]+'.json'),dict(calls=result['calls'],rows=captured))
    for r in captured:
        if 4<=r['t']<=7:
            b=r['band'];t=b['target']
            print(r['t'],{k:b[k] for k in ('supported','front_unobserved','current_occupied_returns','return_behind_lead')},
                {k:t.get(k) for k in ('ahead_clear','lost_advantage_scans','observed_speed_upper_mps')})

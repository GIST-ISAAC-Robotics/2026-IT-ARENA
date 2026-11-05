"""r10의 실제 정체 장면을 같은 입력으로 재생해 후퇴 판정 시각과 실효 명령을 비교한다."""
from collections import Counter
import json
import sys
from pathlib import Path
from arena_autonomy.follow_trace import replay_events
from review import lines,read

HERE=Path(__file__).resolve().parent
def main(name):
    dest=HERE/(name+'.json')
    if dest.exists():raise FileExistsError(dest)
    run=HERE/'closed_loop_v3/run1';tg=read(run/'report.json')['case_window']['green_sim_s']
    old={}
    def events():
        for e in lines(run/'follow_decisions.jsonl'):
            if e['kind']=='call':old[e['seq']]=e['data']['output'][0]
            yield e
    res=replay_events(events());reasons=Counter();abort=[];rows=[]
    for r in res['outcomes']:
        t=r['reference_s']-tg
        if not 0<=t<=14:continue
        m=r['details'].get('moving_pass',{})
        reasons[m.get('reason','active')]+=1
        if 'abort' in m:abort.append(dict(t=t,speed=r['speed_mps'],detail=m))
        if t>=5:
            rows.append((old[r['seq']],r['speed_mps']))
    off_trace=HERE.parent/'moving_follow_continuity_v1/closed_loop_v2/run1/follow_decisions.jsonl'
    off=replay_events(lines(off_trace))
    out=dict(scope='fixed_sensor_and_vehicle_pose_only_no_closed_loop_pass_claim',
        calls=res['calls'],reasons=reasons,aborts=abort,after_five_calls=len(rows),
        old_zero=sum(x<=0 for x,y in rows),new_zero=sum(y<=0 for x,y in rows),
        old_average=sum(x for x,y in rows)/len(rows),new_average=sum(y for x,y in rows)/len(rows),
        off_calls=off['calls'],off_mismatches=len(off['mismatches']))
    with dest.open('x',encoding='utf-8') as f:json.dump(out,f,indent=1);f.write('\n')
    print(out)
    if not off['passed']:raise SystemExit(1)
if __name__=='__main__':main(sys.argv[1])

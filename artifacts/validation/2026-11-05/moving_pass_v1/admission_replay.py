"""같은 중앙 추종 센서 입력에서 추월 ON/OFF가 실제 명령을 변경하는지 대조."""
import json
import sys
from collections import Counter
from pathlib import Path
from arena_autonomy.follow_trace import replay_events
from review import lines
HERE=Path(__file__).resolve().parent
def main(name):
    trace=HERE.parent/'moving_follow_continuity_v1/closed_loop_v2/run1/follow_decisions.jsonl'
    expected={}
    def feed(enabled):
        for e in lines(trace):
            if e['kind']=='header':e['data']['execution_config']['moving_pass_trial']=enabled
            if e['kind']=='call':expected[e['seq']]=e['data']['output'][:2]
            yield e
    off=replay_events(feed(False));on=replay_events(feed(True));diff=[];decisions=Counter();phases=Counter()
    for row in on['outcomes']:
        v,d=expected[row['seq']]
        if abs(v-row['speed_mps'])>1e-10 or abs(d-row['steering_rad'])>1e-10:diff.append(row['seq'])
        m=row['details'].get('moving_pass',{})
        decisions[m.get('entry',{}).get('reason','not_requested')]+=1
        if 'phase' in m:phases[m['phase']]+=1
    out=dict(scope='stored_central_lead_follow_inputs_not_closed_loop',off_calls=off['calls'],
             off_mismatches=len(off['mismatches']),on_calls=on['calls'],actual_command_differences=diff,
             admission=decisions,active=phases)
    with (HERE/(name+'.json')).open('x',encoding='utf-8') as f:json.dump(out,f,indent=1);f.write('\n')
    print(out)
    if not off['passed']:raise SystemExit(1)
if __name__=='__main__':main(sys.argv[1])

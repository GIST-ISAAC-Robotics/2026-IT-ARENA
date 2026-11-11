"""같은 시설 재출발 각본의 r62/r63 실제 결과를 대조. 원 판정은 보존한다."""
import hashlib
import json
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
PREVIOUS=ROOT/'artifacts/validation/2026-11-10/restart_recessed_facility_v1'


def main():
    results={};sources={}
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    for key,folder in [('before',PREVIOUS),('current',A)]:
        d=folder/'moving_stop_restart'; run=d/'run1'
        files=[d/'assessment.json',d/'continuity.json',run/'follow_decisions.jsonl',run/'report.json']
        sources.update({p.relative_to(ROOT).as_posix():sha(p) for p in files})
        assessment=json.loads(files[0].read_bytes());continuity=json.loads(files[1].read_bytes())
        applications=[];reasons={};observed_scans=0
        for line in files[2].open():
            event=json.loads(line)
            if event['kind']!='call':continue
            i,out=event['data']['input'],event['data']['output']
            proposal=out[2].get('bypass',{}).get('observed_entry_return',{})
            if proposal:
                observed_scans+=1
                reasons[proposal['reason']]=reasons.get(proposal['reason'],0)+1
            if proposal.get('applied'):
                applications.append(dict(t=i['reference_s'],command=out[:2],detail=proposal))
        results[key]=dict(passed=assessment['passed'],checks=assessment['checks'],
            goal_gates=assessment['goal']['gates'],metrics=assessment['goal']['metrics'],
            observed_return_evaluated_scans=observed_scans,return_reasons=reasons,applications=applications,
            low_speed_intervals=continuity['low_speed_intervals'],
            abrupt_command_drops=continuity['abrupt_command_drops'],
            largest_deceleration_windows=continuity['deceleration_windows'][:3])
    result=dict(scope='one closed-loop run per version; no hardware or all-condition claim',
        results=results,sources=sources,unchanged=all(sha(ROOT/p)==h for p,h in sources.items()))
    with (A/'motion_comparison.json').open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(unchanged=result['unchanged'],results={k:dict(passed=v['passed'],
        metrics=v['metrics'],applications=[a['t'] for a in v['applications']]) for k,v in results.items()})))

if __name__=='__main__':main()

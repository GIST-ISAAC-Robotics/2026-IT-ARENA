"""같은 코너의 사전 기준은 유지하고 주행 손실·실제 연결 사용을 보충한다."""
from collections import Counter
import hashlib
import json
from pathlib import Path

A=Path(__file__).resolve().parent
PREVIOUS=A.parent/'corner_parallel_continuity_v1'


def describe(folder):
    read=lambda p:json.loads(p.read_text())
    report=read(folder/'run1/report.json')
    continuity=read(folder/'continuity.json')
    assessment=read(folder/'assessment_run1.json')
    counts=Counter();accepted=[];stops=[]
    source=folder/'run1/follow_decisions.jsonl'
    before=hashlib.sha256(source.read_bytes()).hexdigest()
    with source.open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']!='call':continue
            data=event['data'];v,a,detail=data['output'];normal=detail.get('normal') or detail
            counts['calls']+=1
            check=normal.get('selector_confirmed_fringe_recheck') or {}
            if check:
                counts['confirmed_rechecks']+=1
                counts['confirmed_accepted']+=bool(check['accepted'])
                if check['accepted']:
                    accepted.append(dict(t=data['input']['reference_s'],command_mps=v,steering_rad=a,
                        measured_mps=check['measured_speed_mps'],region=check['region']))
            if v<=.001:counts['zero_'+str(normal.get('selector_reason') or normal.get('reason'))]+=1
    assert hashlib.sha256(source.read_bytes()).hexdigest()==before
    return dict(assessment=assessment, progress_m=continuity['progress_m'],
        peak_mps=continuity['peak_speed_mps'], low_intervals=continuity['low_speed_intervals'],
        longest_low_s=max((x['duration_s'] for x in continuity['low_speed_intervals']),default=0),
        command_abrupt_drops=continuity['abrupt_command_drops'],
        counts=dict(counts),accepted=accepted,source_sha256=before,
        stop=report['stop_checks'],shutdown_clean=report['shutdown_clean'])


def main():
    out=A/'motion_comparison.json'
    assert not out.exists()
    result=dict(scope='same scenario, independent run; phase may differ; no revised gates or hardware inference',
        before=describe(PREVIOUS),current=describe(A))
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:{q:v[q] for q in ('progress_m','longest_low_s','counts','shutdown_clean')}
                      for k,v in result.items() if isinstance(v,dict)}))


if __name__=='__main__':main()

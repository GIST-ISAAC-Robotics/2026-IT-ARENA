"""r8 추종 저장 입력에서 추월 OFF 동일성과 ON 진입 후보를 비교. 폐루프 주행 아님."""
from collections import Counter
from pathlib import Path
import argparse
import hashlib
import json

from arena_autonomy.follow_trace import replay_events

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
TRACE=HERE.parent/'moving_follow_continuity_v1/closed_loop_v2/run1/follow_decisions.jsonl'

def events(enabled):
    for line in TRACE.open(encoding='utf-8'):
        event=json.loads(line)
        if event['kind']=='header' and enabled:
            event['data']['execution_config']['moving_pass_trial']=True
        yield event

def main():
    parser=argparse.ArgumentParser();parser.add_argument('name');args=parser.parse_args()
    target=HERE/(args.name+'.json')
    if target.exists(): raise FileExistsError(target)
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
    off=replay_events(events(False));on=replay_events(events(True))
    samples=[];previous=None
    for row in on['outcomes']:
        m=row['details'].get('moving_pass',{})
        entry=m.get('entry',{})
        key=(row['details'].get('reason'),m.get('reason'),m.get('phase'),entry.get('reason'))
        if key!=previous:
            samples.append(dict(seq=row['seq'],time=row['reference_s'],speed=row['speed_mps'],
                steer=row['steering_rad'],reason=key,moving=m))
        previous=key
    report=dict(scope='open_loop_recorded_follow_inputs_not_pass_demonstration',
        trace=str(TRACE.relative_to(ROOT)),trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
        source_sha256=hashes,off=dict(calls=off['calls'],mismatches=len(off['mismatches'])),
        on=dict(calls=on['calls'],changed=len(on['mismatches']),
            reasons=dict(Counter(r['details'].get('moving_pass',{}).get('reason','active') for r in on['outcomes'])),
            phases=dict(Counter(r['details'].get('moving_pass',{}).get('phase','none') for r in on['outcomes']))),
        transitions=samples)
    report['sources_unchanged']=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    target.write_text(json.dumps(report,indent=1,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('off','on','sources_unchanged')},ensure_ascii=False))
    if not off['passed'] or not report['sources_unchanged']: raise SystemExit(1)

if __name__=='__main__':main()

"""두 실행의 같은 각본 시간대만 집계한다. 실차/반복 통계가 아니다."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import sys


def measure(directory):
    report = json.loads((directory/'report.json').read_text())
    rows = [json.loads(s) for s in (directory/'trajectory.jsonl').open()]
    active = [r for r in rows if r.get('elapsed_since_green_s') is not None]
    phases = {}
    for name, start, end in [('lead_moves',0.,4.), ('lead_stops',4.,9.), ('lead_restarts',9.,15.)]:
        observed = quiet = distance = 0.
        for a,b in zip(active, active[1:]):
            ta,tb = a['elapsed_since_green_s'],b['elapsed_since_green_s']
            dt = min(end,tb)-max(start,ta)
            if dt <= 0 or not 0 < tb-ta <= .25:
                continue
            observed += dt
            quiet += dt * (a['ground_speed_mps'] < .05)
            distance += (b['progress_m']-a['progress_m']) * dt/(tb-ta)
        gaps = [r['sampled_gap_m'] for r in active if start<=r['elapsed_since_green_s']<end]
        phases[name] = dict(observed_s=observed,ground_below_005_s=quiet,progress_m=distance,
                            min_trace_gap_m=min(gaps,default=None))
    calls = Counter()
    with (directory/'follow_decisions.jsonl').open() as stream:
        for line in stream:
            r=json.loads(line)
            if r['kind'] != 'call':
                continue
            detail=r['data']['output'][2]
            calls['all'] += 1
            reserve=detail.get('follow_entry_space',{})
            if reserve.get('reason')=='advancing_patch_follow':
                calls['moving_patch_seen'] += 1
                before, fixed = reserve['before_mps'], reserve['fixed_reserve_cap_mps']
                if min(before,reserve['cap_mps']) > min(before,fixed)+1e-8:
                    calls['moving_branch_raised_request_before_later_caps'] += 1
            if reserve.get('applied'):
                calls['applied:'+str(reserve.get('reason'))] += 1
    ev=report['evaluation']
    return dict(passed=report['passed'],valid=report['run_valid'], header=report.get('handoff_execution'),
                gates=report['gates'], stop=report['stop_checks'], gap=ev['gap'], road=ev['road'],
                progress=ev['progress'], hold_restart=ev['case_metrics'], phases=phases, calls=calls,
                sources=report.get('source_unchanged'), trajectory_sha256=hashlib.sha256(
                    (directory/'trajectory.jsonl').read_bytes()).hexdigest())


if __name__=='__main__':
    paths=[Path(p) for p in sys.argv[1:]]
    if len(paths)!=2:
        raise SystemExit('usage: compare.py FIXED_RUN MOVING_RUN')
    pair={key:measure(path) for key,path in zip(('fixed','moving'),paths)}
    for arm in pair:
        if pair[arm]['header']['reserve']!=arm:
            raise ValueError('arm/header mismatch')
    headers=[]
    for path in paths:
        with (path/'follow_decisions.jsonl').open() as stream:
            h=json.loads(next(stream))['data']
        headers.append(h)
    from copy import deepcopy
    normalized=deepcopy(headers[1])
    normalized['execution_config']['moving_follow_reserve']=False
    pair['same_header_except_moving_reserve'] = headers[0]==normalized
    if not pair['same_header_except_moving_reserve']:
        raise ValueError('recorded configurations differ beyond moving_follow_reserve')
    print(json.dumps(pair,ensure_ascii=False,indent=2))

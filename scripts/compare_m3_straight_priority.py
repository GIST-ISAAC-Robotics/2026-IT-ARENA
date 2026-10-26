"""좌우 피추월·정지/재개 저장 센서에서 직진 우선 OFF/ON 비교. 새 주행 증거가 아니다."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile

import numpy as np

from arena_autonomy.follow_trace import freeze, replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import ExecutionConfig, M3ExecutionFollow
from probe_grid_follow import ROOT


def replay(events, enabled):
    header = events[0]['data']
    config = replace(ExecutionConfig(**header['execution_config']), straight_with_threat=enabled)
    controller = M3ExecutionFollow(FollowConfig(**header['follow_config']), config, allow_simulation_trial=True)
    motion = MotionConfig(**header['motion_config'])
    rows = []
    for event in events[1:-1]:
        if event['kind'] == 'reset':
            controller.reset()
            rows.append(dict(reset=True))
        elif event['kind'] == 'call':
            start = time.perf_counter_ns()
            output = freeze(replay_call(controller, event['data']['input'], motion))
            rows.append(dict(output=output, reference_s=event['data']['input']['reference_s'],
                cost_ms=(time.perf_counter_ns()-start)/1e6))
        else:
            raise ValueError('unexpected stored event')
    return rows


def compare(off, on):
    if len(off) != len(on): raise ValueError('event alignment lost')
    different = []
    nonthreat_changes, return_changes, record_changes = [], [], []
    for index, (a, b) in enumerate(zip(off,on)):
        if a.get('reset'):
            assert b.get('reset')
            continue
        old, new = a['output'], b['output']
        change = old[:2] != new[:2] or old[2]['reason'] != new[2]['reason']
        if change: different.append(dict(index=index,t_s=a['reference_s'],old=old[:2],new=new[:2],
            old_reason=old[2]['reason'],new_reason=new[2]['reason']))
        oa, ob = old[2].get('m3_lateral',{}), new[2].get('m3_lateral',{})
        has_threat = any(v['present'] for v in oa.get('threats',{}).values())
        if not has_threat and change: nonthreat_changes.append(index)
        state = oa.get('state',{})
        if not has_threat and abs(state.get('d0',0)) >= .08 and change: return_changes.append(index)
        if oa.get('release_gate') != ob.get('release_gate') or oa.get('threats') != ob.get('threats'):
            record_changes.append(index)
    metrics = []
    for variant, rows in (('off',off),('on',on)):
        calls = [r for r in rows if not r.get('reset')]
        times = [r['cost_ms'] for r in calls]
        metrics.append(dict(variant=variant,calls=len(calls),resets=len(rows)-len(calls),
            positive=sum(r['output'][0]>0 for r in calls),
            reasons=dict(Counter(r['output'][2]['reason'] for r in calls)),
            selected=dict(Counter(r['output'][2].get('m3_candidate','none') for r in calls)),
            p95_ms=float(np.percentile(times,95)),max_ms=max(times)))
    return dict(metrics=metrics,changed_actions=different,nonthreat_changed_calls=nonthreat_changes,
        large_return_changed_calls=return_changes,threat_or_release_gate_changes=record_changes)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); output=args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'): parser.error('new validation directory required')
    output.mkdir(parents=True,exist_ok=False)
    names=['run17_observed_release_closed_loop','run21_right_encounter','run19_stop_restart']
    paths=[ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1'/n/'run/follow_decisions.jsonl' for n in names]
    sources={Path(__file__)} | set((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    for module in tuple(sys.modules.values()):
        value=getattr(module,'__file__',None)
        if value:
            path=Path(value).resolve()
            if path.is_relative_to(ROOT) and path.suffix=='.py':sources.add(path)
    frozen={p.relative_to(ROOT).as_posix():p.read_bytes() for p in sorted(sources)}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for n,b in frozen.items():archive.writestr(n,b)
    details, summaries = {}, {}
    for name,path in zip(names,paths):
        raw=path.read_bytes(); events=[json.loads(line) for line in raw.decode('utf-8').splitlines()]
        off,on=replay(events,False),replay(events,True)
        report=compare(off,on)
        expected_path=ROOT/'artifacts/validation/2026-10-26/m3_rear_residual_v1/cachefix_runs/run7_premises_cachefix'/(
            name+'__plain.json')
        expected_raw=expected_path.read_bytes(); expected=json.loads(expected_raw)
        differences=[]
        if len(off)!=len(expected):raise ValueError('expected event alignment lost')
        for i,(row,record) in enumerate(zip(off,expected)):
            if row.get('reset'):
                match=record=={'reset':True}
            else:
                digest=hashlib.sha256(json.dumps(row['output'],sort_keys=True).encode()).hexdigest()
                match=digest==record['sha']
            if not match:differences.append(i)
        report.update(off_expected_differences=differences,
            expected_sha256=hashlib.sha256(expected_raw).hexdigest(),
            expected_unchanged=expected_path.read_bytes()==expected_raw)
        report.update(input_sha256=hashlib.sha256(raw).hexdigest(),input_unchanged=raw==path.read_bytes())
        details[name]=dict(off=off,on=on,comparison=report)
        summaries[name]={**report,'changed_actions_count':len(report['changed_actions']),
            'changed_actions':report['changed_actions'][:6]}
        print(json.dumps({name:summaries[name]}),flush=True)
    unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items())
    common=dict(source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
        sources_unchanged=unchanged,driving_authorized=False,
        scope='fixed stored sensor inputs; changed commands do not update next sensor scene')
    (output/'report.json').write_text(json.dumps(dict(common,cases=details),ensure_ascii=False),encoding='utf-8')
    (output/'summary.json').write_text(json.dumps(dict(common,cases=summaries),ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if unchanged and all(c['input_unchanged'] and c['expected_unchanged'] and
        not c['off_expected_differences'] for c in summaries.values()) else 1


if __name__=='__main__':raise SystemExit(main())

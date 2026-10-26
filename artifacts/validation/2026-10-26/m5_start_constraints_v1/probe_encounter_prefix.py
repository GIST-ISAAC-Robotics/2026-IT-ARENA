"""새 전체 옵션이 기존 초기 측방 간격 실패 이전 행동을 바꾸는지 먼저 확인한다."""
from dataclasses import replace
import argparse
import hashlib
import json
from pathlib import Path
import zipfile
from arena_autonomy.follow_trace import freeze,replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import ExecutionConfig,M3ExecutionFollow

ROOT=Path(__file__).resolve().parents[4]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'run48_encounter_prefix')
OUT=parser.parse_args().output.resolve()
if not OUT.is_relative_to(ROOT/'artifacts/validation'):parser.error('new validation output required')
OUT.mkdir(exist_ok=False)
sources={Path(__file__)}|set((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
frozen={p.relative_to(ROOT).as_posix():p.read_bytes() for p in sources}
with zipfile.ZipFile(OUT/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
    for n,b in frozen.items():z.writestr(n,b)
results={}
for name in ('run17_observed_release_closed_loop','run21_right_encounter'):
    path=ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1'/name/'run/follow_decisions.jsonl'
    raw=path.read_bytes();events=[json.loads(line) for line in raw.decode().splitlines()]
    header=events[0]['data'];motion=MotionConfig(**header['motion_config'])
    old=ExecutionConfig(**header['execution_config'])
    new=replace(old,small_center_straight=True,straight_surface_commitment=True,rounded_occupancy=True,
                straight_with_threat=True,defer_center_on_straight=True)
    controllers=[M3ExecutionFollow(FollowConfig(**header['follow_config']),e,allow_simulation_trial=True) for e in (old,new)]
    refpath=ROOT/'artifacts/validation/2026-10-26/m3_rear_residual_v1/cachefix_runs/run7_premises_cachefix'/(name+'__plain.json')
    refraw=refpath.read_bytes();reference=json.loads(refraw)
    differences=[];actions=[];count=0;resets=0
    for i,event in enumerate(events[1:-1]):
        if event['kind']=='reset':
            for ctl in controllers:ctl.reset()
            resets+=1
            if reference[i]!={'reset':True}:differences.append(i)
            continue
        if event['kind']!='call':raise ValueError('unexpected event')
        t=event['data']['input']['reference_s']
        if t>6.:break
        a,b=[freeze(replay_call(ctl,event['data']['input'],motion)) for ctl in controllers]
        count+=1
        if hashlib.sha256(json.dumps(a,sort_keys=True).encode()).hexdigest()!=reference[i]['sha']:differences.append(i)
        if a[:2]!=b[:2] or a[2]['reason']!=b[2]['reason']:
            actions.append(dict(index=i,t_s=t,old=a[:2],new=b[:2],old_reason=a[2]['reason'],
                new_reason=b[2]['reason'],old_candidate=a[2].get('m3_candidate'),new_candidate=b[2].get('m3_candidate'),
                old_clearance=a[2].get('swept_clearance_m'),new_clearance=b[2].get('swept_clearance_m')))
    results[name]=dict(calls=count,resets=resets,baseline_differences=differences,action_differences=actions,
        changed_positive_yield=sum(a['old'][0]>0 and a['old_candidate']=='yield' and a['new_candidate']!='yield' for a in actions),
        input_unchanged=path.read_bytes()==raw,expected_unchanged=refpath.read_bytes()==refraw,
        input_sha256=hashlib.sha256(raw).hexdigest(),expected_sha256=hashlib.sha256(refraw).hexdigest())
    print(json.dumps({name:results[name]}),flush=True)
unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items())
passed=unchanged and all(not r['baseline_differences'] and r['input_unchanged'] and r['expected_unchanged'] for r in results.values())
(OUT/'summary.json').write_text(json.dumps(dict(cases=results,sources_unchanged=unchanged,integrity_pass=passed,
    source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
    scope='fixed inputs up to 6s; early response comparison only, no trajectory or safety claim'),ensure_ascii=False,indent=2),encoding='utf-8')
raise SystemExit(0 if passed else 1)

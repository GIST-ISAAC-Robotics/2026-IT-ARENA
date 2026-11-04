"""격리한 실제 제어기 OFF는 원래 출력과, ON은 독립 속도 계산과 대조한다."""
import csv
import hashlib
import json
import sys
from dataclasses import replace
from pathlib import Path
from time import perf_counter
from zipfile import ZipFile, ZIP_DEFLATED

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
STAGE=ROOT/'build/solo_speed_envelope_stage'


def digest(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f,'sha256').hexdigest()


def register():
    files=sorted([p for p in STAGE.rglob('*.py') if p.is_file()])
    entries={p.relative_to(STAGE).as_posix():digest(p) for p in files}
    with ZipFile(HERE/'staged_source.zip','x',ZIP_DEFLATED) as z:
        for name in entries:
            z.write(STAGE/name,name)
    with (HERE/'staged_source.json').open('x',encoding='utf8') as f:
        json.dump(entries,f,indent=2)
    return entries


def run(mode):
    entries=json.loads((HERE/'staged_source.json').read_bytes())
    assert all(digest(STAGE/name)==h for name,h in entries.items())
    sys.path[:0]=[str(STAGE/'src/arena_autonomy'),str(STAGE/'src/arena_vehicle_interface'),str(ROOT/'tests')]
    from arena_autonomy.local_follow import LocalFollow,FollowConfig
    if mode=='tests':
        import pytest
        return pytest.main(['-q',str(STAGE/'tests/test_solo_straight_budget.py')])
    from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
    from arena_autonomy.lidar_motion import MotionConfig
    import numpy as np
    source=HERE.parent/'solo_post_free_v1/run1/follow_decisions.jsonl'
    with (HERE/'post_free_calls.csv').open(encoding='utf8') as f:
        expected={float(r['t']):r for r in csv.DictReader(f)}
    seq=0;calls=0;fail=[];nerr=0;checked=0;cost=[];changed=0;footer=None
    for event in map(json.loads,source.open(encoding='utf8')):
        assert footer is None and event['seq']==seq
        seq+=1;data=event['data']
        if event['kind']=='header':
            cfg=replace(FollowConfig(**data['follow_config']),solo_straight_budget_trial=(mode=='on'))
            ctrl=LocalFollow(cfg);motion=MotionConfig(**data['motion_config'])
        elif event['kind']=='reset':
            ctrl.reset()
        elif event['kind']=='call':
            begin=perf_counter()
            out=freeze(replay_call(ctrl,data['input'],motion));cost.append((perf_counter()-begin)*1000)
            calls+=1;t=data['input']['reference_s'];orig=data['output'];row=expected.get(t)
            if mode=='off':
                good=equal_output(comparable_decision(out),comparable_decision(orig))
            else:
                good=(out[1]==orig[1] and out[2].get('reason')==orig[2].get('reason')
                      and out[2].get('behavior')==orig[2].get('behavior') and out[0]>=orig[0]-1e-8
                      and (out[0]<=1e-9)==(orig[0]<=1e-9))
                if row is not None:
                    checked+=1
                    good=good and abs(out[0]-float(row['new']))<1e-8 and out[2]['solo_straight_budget']==row['why']
                changed+=out[0]>orig[0]+1e-8
            if not good:
                nerr+=1
                if len(fail)<8:
                    fail.append(dict(t=t,actual=out[:2],old=orig[:2],why=out[2].get('solo_straight_budget'),expected=row))
        elif event['kind']=='footer':
            footer=data
    assert footer and footer['written']==footer['accepted']==seq-1
    assert not any(footer.get(k) for k in ('pending','worker_alive','dropped','failed','capture_errors'))
    assert mode!='on' or checked==len(expected)
    modules=[Path(m.__file__).resolve() for name,m in sys.modules.items()
             if name.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(m,'__file__',None)]
    assert modules and all(f.is_relative_to(STAGE) for f in modules)
    result=dict(mode=mode,calls=calls,sum_changed_including_after_stop=changed if mode=='on' else None,
                mismatches=nerr,first_mismatches=fail,independent_speed_window_calls_checked=checked,
                isolated_modules=len(modules),source_sha256=digest(source),
                replay_call_cost_ms=dict(median=float(np.median(cost)),p95=float(np.percentile(cost,95)),maximum=max(cost)),
                scope='같은 원래 측정·직전 조향에서 제어기 판단을 재생했다. ROS나 실제 궤적의 성능 증명이 아니다.')
    with (HERE/('staged_'+mode+'.json')).open('x',encoding='utf8') as f:
        json.dump(result,f,indent=2,ensure_ascii=False,allow_nan=False)
    print(result)
    return int(nerr>0)


if __name__=='__main__':
    mode,=sys.argv[1:]
    if mode=='register':
        print(len(register()),'files registered')
    elif mode in ('tests','off','on'):
        raise SystemExit(run(mode))
    else:
        raise ValueError(mode)

"""r344 최초 코너 정체를 고정 입력에서 분해한다. 허가/주행 연결 변경 없음."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
spec = importlib.util.spec_from_file_location('r342_helpers', A.parent/'current_positive_cap_r342_v1/analyze.py')
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
old = helpers.old
spec = importlib.util.spec_from_file_location('r343_corner_audit', A.parent/'current_solo_low_r343_v1/candidate.py')
candidate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.released_body_gap import check


def main():
    assert not (A/'report.json').exists()
    source = A.parent/'current_solo_low_drive_r344_v1/run1/follow_decisions.jsonl'
    with source.open() as stream:
        header = json.loads(next(stream))['data']
        calls = [e['data'] for e in map(json.loads, stream)
                 if e['kind'] == 'call' and 'current_rebase' in e['data']]
    unknown = [i for i,r in enumerate(calls) if r['current_rebase']['output']['reason'] ==
               'route_current_large_motion_region_unobserved']
    selected = sorted(set(unknown[int(i)] for i in np.linspace(0,len(unknown)-1,12,dtype=int)))
    cfg = old.FollowConfig(**header['follow_config'])
    motion = old.MotionConfig(**header['motion_config'])
    lateral = ps.lateral_config(cfg)
    selector = ps.SelectorConfig()
    old.configure('native_v1')
    rows, evidence = [], []
    for index in selected:
        record = calls[index]
        current = record['current_rebase']['input']
        d, scan, history = old.restore_call(current, motion)
        scene = old.motion_scene(scan, history, d['reference_s'], timing_verified=d['timing_verified'],
                                epoch=d['epoch'], rear_half_angle_deg=d['rear_half_angle_deg'])
        replay = old.freeze(candidate.rebase(record['output'], old.plan_reference(record['input']),
            current, scene, history, cfg, require_current_region=True))
        expected = record['current_rebase']['output']
        assert old.equal_output(replay, expected), (index, 'full output differs')
        trials = []
        for cap in sorted(set([record['output'][0], .35, .15, .08]), reverse=True):
            if not 0 < cap <= record['output'][0]:
                continue
            low = (cap,record['output'][1],record['output'][2])
            checked = candidate.rebase(low, old.plan_reference(record['input']), current,
                                       scene, history, cfg, require_current_region=True)
            context = {}
            region = ps.motion_region(scene,d['speed_mps'],cap,checked['steering_rad'],
                d['previous_steering_rad'],d['dt_s'],cfg,lateral,context=context)
            q = context['queries'][~context['free'] & ~context['masked']] if context else np.empty((0,2))
            bounds = dict(min_xy=q.min(axis=0).tolist(),max_xy=q.max(axis=0).tolist()) if len(q) else {}
            bounded = check(scene,scan,d['speed_mps'],cap,checked['steering_rad'],
                            d['previous_steering_rad'],d['dt_s'],cfg,lateral,selector)
            trials.append(dict(cap_mps=cap,actual_target_mps=checked['speed_mps'],reason=checked['reason'],
                region=region,unknown_bounds=bounds,bounded_diagnostic=old.freeze(bounded)))
        rows.append(dict(index=index,reference_s=d['reference_s'],measured_speed_mps=d['speed_mps'],
            plan_speed_mps=record['output'][0],plan_reason=record['output'][2]['normal'].get('reason'),
            plan_partial=record['output'][2]['normal'].get('partial_observation',{}).get('reason'),
            original_full_equal=True,trials=trials))
        evidence.append(record)
    sources={source.relative_to(ROOT).as_posix():hashlib.sha256(source.read_bytes()).hexdigest()}
    for module in list(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            p=Path(name).resolve()
            if p.is_file() and p.is_relative_to(ROOT) and p.suffix in ('.py','.so'):
                sources[p.relative_to(ROOT).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
    (A/'report.json').write_text(json.dumps(dict(scope='fixed inputs only; no policy adoption',
        original_calls=len(calls),unknown_calls=len(unknown),selected=selected,source_hashes=sources,rows=rows),indent=2)+'\n')
    (A/'evidence.json').write_text(json.dumps(evidence,separators=(',',':'))+'\n')
    print(json.dumps([dict(index=r['index'],speed=r['measured_speed_mps'],
        trials=[dict(cap=t['cap_mps'],target=t['actual_target_mps'],unknown=t['region']['unobserved'],
                     bounded=t['bounded_diagnostic'].get('proof',{}).get('reason',t['bounded_diagnostic'].get('reason')))
                for t in r['trials']]) for r in rows]))


if __name__ == '__main__':
    main()

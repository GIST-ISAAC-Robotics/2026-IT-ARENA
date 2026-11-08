"""현재 정지 입력의 긴 선호 경로와 짧은 반대 관측 비교. 런타임 수정 없는 설계 탐색."""
from dataclasses import replace
import inspect
import json
from pathlib import Path

import numpy as np

from arena_autonomy import corner_corridor as cc
from arena_autonomy.continuous_prefix import refine
from arena_autonomy.local_follow import FollowConfig

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent/'piecewise_corner_v1'
header = json.loads((SOURCE/'run1/follow_decisions.jsonl').open().readline())['data']
cfg = FollowConfig(**header['follow_config'])

# 메모리에만 다른 역할 조합을 추가한다. 공용 함수/파일에는 적용하지 않는다.
text = inspect.getsource(cc.short_prefix_evidence)
text = text.replace('def short_prefix_evidence(points, side, config):',
                    'def candidate(points, side, config, allow_long_preferred=False):')
old = "    if info['boundary_observed_ahead_m'] >= config.boundary_min_ahead_m:\n        return {**base, 'reason':'not_a_short_boundary'}"
new = """    long_preferred = info['boundary_observed_ahead_m'] >= config.boundary_min_ahead_m
    if long_preferred:
        if not allow_long_preferred or other_info['boundary_observed_ahead_m'] >= config.boundary_min_ahead_m:
            return {**base, 'reason':'not_a_short_boundary'}
        if abs(other_info['wall_distance_m']-config.wall_offset_m) > config.boundary_anchor_error_m:
            return {**base, 'reason':'opposite_anchor_unconfirmed'}
        base['long_preferred_short_opposite'] = True
        base['base_observed_ahead_m'] = other_info['boundary_observed_ahead_m']"""
assert old in text
text = text.replace(old, new).replace(
    "if other_info['boundary_observed_ahead_m'] < config.boundary_min_ahead_m:",
    "if not long_preferred and other_info['boundary_observed_ahead_m'] < config.boundary_min_ahead_m:")
namespace = dict(cc.__dict__)
exec(compile(text, '<paired-prefix-shadow>', 'exec'), namespace)
candidate = namespace['candidate']
clouds = json.loads((SOURCE/'corner_after.clouds.json').read_text())
rows = []
for sample in clouds:
    points = np.asarray(sample['calls'][0]['points'])
    for mirrored in (False, True):
        p = points.copy()
        if mirrored:p[:, 1] *= -1
        side = 'right' if mirrored else sample['side_in']
        old = cc.short_prefix_evidence(p, side, cfg)
        new = refine(candidate(p, side, cfg, True), cfg)
        keep = ('supported','reason','shared_span_m','one_sided_lead_in_m','base_observed_ahead_m',
                'retained_ahead_m','long_preferred_short_opposite','continuous_prefix')
        rows.append(dict(t=sample['t'],side=side,mirrored=mirrored,old_reason=old['reason'],
                         proposed={k:v for k,v in new.items() if k in keep}))
(HERE/'shadow_function.txt').write_text(text, encoding='utf-8')
(HERE/'shadow_probe.json').write_text(json.dumps(dict(scope='rounded stored points, geometry only; no runtime connection',
    samples=rows),indent=2),encoding='utf-8')

def artificial(own, opposite, own_ahead=1.4, other_ahead=.46, other_anchor=.505):
    def local_path(points, side, offset):
        path = own if side == 'left' else opposite
        return path, dict(boundary_observed_ahead_m=own_ahead if side == 'left' else other_ahead,
                         wall_distance_m=.505 if side == 'left' else other_anchor, path_points=path.tolist())
    actual = namespace['local_path']
    namespace['local_path'] = local_path
    try:
        return refine(candidate(np.array([[0., 0.]]), 'left', cfg, True), cfg)
    finally:
        namespace['local_path'] = actual


def line(start, end, y=0., n=36):
    return np.c_[np.linspace(start,end,n), np.full(n,y)]


own, opposite = line(-.2,1.4), line(.14,.60)
cases = dict(
    common_prefix=(artificial(own,opposite), True),
    too_far_lead=(artificial(own,line(.40,.95)),False),
    lateral_disagreement=(artificial(own,line(.14,.60,.15)),False),
    wrong_opposite_anchor=(artificial(own,opposite,other_anchor=.75),False),
    both_long=(artificial(own,opposite,other_ahead=1.),False),
    displaced_origin=(artificial(line(-.2,1.4,.20),line(.14,.60,.20)),False),
    disconnected_islands=(artificial(own,np.vstack((line(.12,.17),np.array([[.20,.30],[.55,.30]]),line(.60,.95)))),False))
checks={name:dict(expected=expected,supported=r['supported'],reason=r['reason']) for name,(r,expected) in cases.items()}
(HERE/'counterexamples.json').write_text(json.dumps(checks,indent=2),encoding='utf-8')
assert all(r['supported'] == expected for r,expected in cases.values()), checks
print(json.dumps(dict(samples=rows,checks=checks)))

"""원 정체 장면의 점유 추가/광선 가림 반례. 별도 overlay만 사용한다."""
from pathlib import Path
from dataclasses import replace
from functools import partial
import json, math, sys, time
import numpy as np

A = Path(__file__).resolve().parent
sys.path.insert(0, str(A/'overlay'))
from arena_autonomy.follow_trace import restore_call, motion_scene, MotionConfig, FollowConfig
from arena_autonomy.bypass_entry_trial import _compare_entries, EntryConfig, entry_motion_region
from arena_autonomy.pass_release_trial import lateral_config, region_blocked, SelectorConfig
from arena_autonomy.adaptive_entry_preview import sampled_region_rejection
from arena_autonomy.adaptive_entry_preview import compare_shorter_previews, shorter_pursuit

d = json.loads((A.parent/'ops_traffic_centered_entry_v1/blocked_snapshot.json').read_bytes())
cfg = FollowConfig(**d['follow_config'])
i, scan, history = restore_call(d['input'], MotionConfig(**d['motion_config']))
scene = motion_scene(scan, history, i['reference_s'], timing_verified=i['timing_verified'],
                     epoch=i['epoch'], rear_half_angle_deg=i['rear_half_angle_deg'])
for line in (A.parent/'ops_traffic_centered_entry_v1/run1/follow_decisions.jsonl').open():
    row = json.loads(line)
    if row['kind'] == 'call' and row['data']['input']['reference_s'] >= 30:
        bypass = row['data']['output'][2]['bypass']; break
candidate = bypass['candidates'][0]
guide = dict(reference=bypass['reference'], path=np.asarray(d['path']), offset_m=candidate['offset_m'],
    transition_m=candidate['transition_m'], outer_side='right', stationary_outer_boundary=True)
entry = EntryConfig(offsets_m=(0., -.27, .27), transitions_m=(.6, .8, .9, 1.2))
padding_by_ray = dict(zip((r.source_index for r in scene.rays if r.status == 'return'), d['padding']))

def changed_ray(ray, kind):
    angle = math.atan2(ray.direction_xy[1], ray.direction_xy[0])
    if kind == 'front_occupied' and abs(angle) < math.radians(15):
        # 원 취득 시각과 방향 유지. 앞쪽 가까운 물체를 같은 광선의 반환점으로 넣는다.
        hit = tuple(ray.ray_origin_xy_m[j]+.15*ray.direction_xy[j] for j in (0, 1))
        return replace(ray, hit_xy_m=hit, status='return')
    if kind == 'right_unknown' and math.radians(-100) < angle < math.radians(15):
        return replace(ray, hit_xy_m=None, status='unknown')
    return ray

results = {}; equivalence = []
for kind in ('original_aligned', 'front_occupied', 'right_unknown'):
    altered = replace(scene, rays=tuple(changed_ray(r, kind) for r in scene.rays))
    padding = np.asarray([padding_by_ray.get(r.source_index, max(d['padding']))
                          for r in altered.rays if r.status == 'return'])
    kwargs = dict(config=cfg, entry=entry, guide=guide,
        padding_provider=lambda *args: (padding, dict(scope='same padding or conservative maximum for new return')))
    # 실제 직전각이 도달한 뒤에도 새 점유/미관측은 다시 검사해야 한다.
    previous = -.316443
    primary = _compare_entries(altered, altered.t_ref_s, 0., previous, .02, **kwargs)
    def compare(look, prev):
        return _compare_entries(altered, altered.t_ref_s, 0., prev, .02,
            pursuit_provider=partial(shorter_pursuit, lookahead_m=look), **kwargs)
    begin = time.perf_counter()
    out = compare_shorter_previews(primary, compare, history=history,
        now=altered.t_ref_s, speed=0., previous=previous, dt=.02)
    elapsed = (time.perf_counter()-begin)*1000
    for angle, target in ((-.37,.35),(-.316443,.35),(-.20,.15),(-.1343,.35)):
        args = (altered, 0., target, angle, previous, .02, cfg, lateral_config(cfg), padding)
        sparse = sampled_region_rejection(*args)
        full = entry_motion_region(*args)
        sparse_reason = region_blocked(sparse,SelectorConfig()) if sparse else None
        full_reason = region_blocked(full,SelectorConfig())
        assert sparse_reason is None or full_reason is not None, (kind, angle, sparse, full)
        equivalence.append(dict(case=kind,steering=angle,target=target,
            sampled_rejection=sparse_reason,full_rejection=full_reason))
    results[kind] = dict(selected=out.get('selected'), alignment=out.get('preview_alignment'),
        preview=out.get('adaptive_preview'), reason=out['reason'],
        row_reasons=[r['reason'] for r in out.get('candidates', [])],
        fallback_wall_ms=elapsed, changed_rays=sum(a != b for a,b in zip(altered.rays,scene.rays)))
assert results['original_aligned']['selected'] is not None
assert all(results[k]['selected'] is None and results[k]['alignment'] is None
           for k in ('front_occupied', 'right_unknown'))
out = dict(scope='saved scene perturbation; no simulator run; timing is local wall time under concurrent simulation',
           cases=results, sampled_rejection_vs_full=equivalence)
(A/'counterexample_probe.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(out,ensure_ascii=False))

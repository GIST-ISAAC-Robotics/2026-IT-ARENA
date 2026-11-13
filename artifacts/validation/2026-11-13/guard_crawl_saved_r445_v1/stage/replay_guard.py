"""원 plan을 순차 재생하고 동일성 확인 후 복제 상태에서만 GUARD .1을 진단한다."""
from __future__ import annotations
import collections
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import pickle
import platform
import sys
import time

import numpy as np
from arena_autonomy import pass_release_trial as ps
from arena_autonomy import m3_execution_trial as mx
from arena_autonomy.follow_trace import freeze
from arena_autonomy.race_runtime_policy.evaluator import Evaluator

B = Path(__file__).resolve().parent
O = B / 'result_v1'
O.mkdir(exist_ok=False)
manifest = json.loads((B / 'manifest.json').read_text())
def sha(data): return hashlib.sha256(data).hexdigest()
def changes(): return [n for n, h in manifest.items() if sha((B / n).read_bytes()) != h]
def write(name, obj): (O / name).write_text(json.dumps(obj, ensure_ascii=False, indent=2) + '\n')
def digest(obj): return sha(pickle.dumps(obj, protocol=5))
def clone(obj): return copy.deepcopy(obj)
def same(a, b): return a == b
assert not changes()
trace = [json.loads(line) for line in (B / 'inputs/follow_decisions.jsonl').open()]
assert [r['seq'] for r in trace] == list(range(len(trace)))
assert trace[-1]['kind'] == 'footer' and trace[-1]['data']['capture_errors'] == 0
header = trace[0]['data']
assert header['runtime_policy'] == 'cad_runtime_r377_v1'
original_at_cap = ps.PassReleaseSelector._guard_curve_at_cap
slot = {}
active_seq = None
active_measured = None

def observe_cap(owner, *args, target_cap=None):
    # 첫 원 cap 호출 직전 상태·입력을 복제한다. 후보는 baseline 결과 확인 뒤 실행한다.
    start = time.perf_counter()
    if target_cap is None and not slot.get('snapshot'):
        slot['snapshot'] = clone((owner, args))
    slot['hook_copy_wall_ms'] = slot.get('hook_copy_wall_ms', 0) + 1000 * (time.perf_counter() - start)
    start = time.perf_counter()
    result = original_at_cap(owner, *args, target_cap=target_cap)
    elapsed = 1000 * (time.perf_counter() - start)
    info = (result[2].get('selector_guard_candidate') or {}) if result else {}
    slot.setdefault('caps', []).append(dict(target_cap=target_cap, target_mps=info.get('target_speed_mps'),
        wall_ms=elapsed, result=freeze(result), owner_state=freeze(owner.sides),
        guard_anchor=freeze(owner.guard_anchor), signed_original_measured_mps=active_measured,
        planning_speed_mps=args[4]))
    return result


def first_difference(a, b, path='$'):
    if isinstance(a, dict) and isinstance(b, dict):
        if a.keys() != b.keys(): return dict(path=path, expected_keys=list(b), actual_keys=list(a))
        for k in a:
            if not same(a[k], b[k]): return first_difference(a[k], b[k], path + '.' + str(k))
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b): return dict(path=path, expected_length=len(b), actual_length=len(a))
        for i, (x, y) in enumerate(zip(a, b)):
            if not same(x, y): return first_difference(x, y, path + '[' + str(i) + ']')
    return dict(path=path, expected=b, actual=a)


def qualifies(record):
    caps = record.get('caps', [])
    if not 0 <= active_measured <= .15 or len(caps) != 3: return False
    if [c['target_mps'] for c in caps] != [.35, .25, .15]: return False
    owner, args = record['snapshot']
    if owner.mode != 'GUARD' or args[4] != active_measured: return False
    for c in caps:
        result = c['result']; info = result[2].get('selector_guard_candidate') or {}; region = info.get('region') or {}
        if (result[0] != 0 or info.get('accepted') is not False
                or result[2].get('selector_reason') not in ('guard_curve_rejected_large_motion_masked_toward_left', 'guard_curve_rejected_large_motion_masked_toward_right')
                or region.get('unobserved') != 0 or region.get('masked_front_samples') != 0
                or region.get('queries', 0) <= 0 or region.get('masked_tail_extent_m', 0) <= owner.config.masked_tail_allowance_m):
            return False
    return True


def sensor_geometry(owner, args):
    # 기존 코드가 실제 사용한 반환·경로 투영/compact mask를 읽는다. 정답 위치 사용 없음.
    scene, view, baseline, parallel, speed, previous, dt, age, detail, raw = args
    cfg = owner.lat
    grid = mx.ReturnGrid(scene, cfg)
    ds, masks = mx.side_return_masks(view['frame'], view['state'], grid, cfg)
    _, lateral = view['frame'].project(grid.returns)
    rays = [r for r in scene.rays if r.status == 'return']
    component = np.empty(len(rays), dtype=int)
    first = 0
    for i, group in enumerate(grid.return_components):
        component[first:first + group['points']] = i
        first += group['points']
    assert first == len(rays)
    evidence = dict(config={k:getattr(cfg, k) for k in ('wall_offset_m', 'wall_tolerance_m', 'band_behind_m', 'band_ahead_m', 'min_threat_points')},
        ego_path_state=freeze(view['state']), baseline_path=freeze(baseline[2].get('path_points')),
        baseline_reason=baseline[2].get('reason'), boundary_selection=baseline[2].get('boundary_selection'),
        wall_fallback=baseline[2].get('wall_fallback'), path_wall=baseline[2].get('path_wall'),
        returns_xy=grid.returns.tolist(), source_indices=[r.source_index for r in rays],
        components=grid.return_components, threat_points={}, sensor_only=True, classification_not_changed=True)
    for name, mask in zip(('left', 'right'), masks):
        points = []
        for ix in np.flatnonzero(mask):
            ids = np.arange(max(0, ix - 8), min(len(rays), ix + 9))
            local = grid.returns[ids]
            center = local.mean(axis=0)
            _, _, vt = np.linalg.svd(local-center, full_matrices=False)
            residuals = (local-center) @ vt[-1]
            points.append(dict(return_index=int(ix), source_index=rays[ix].source_index,
                xy=grid.returns[ix].tolist(), path_ds_m=float(ds[ix]), path_lateral_m=float(lateral[ix]),
                expected_wall_offset_distance_m=float(min(abs(lateral[ix]-cfg.wall_offset_m), abs(lateral[ix]+cfg.wall_offset_m))),
                compact_component_index=int(component[ix]), compact_component=grid.return_components[component[ix]],
                neighboring_source_indices=[rays[k].source_index for k in ids], neighbors_xy=local.tolist(),
                neighbor_line_rms_m=float(np.sqrt(np.mean(residuals**2))),
                neighbor_line_max_residual_m=float(np.max(abs(residuals))),
                neighbor_line_direction=vt[0].tolist()))
        evidence['threat_points'][name] = points
    return evidence


started = time.monotonic()
resets = 0
baseline_rows = []
candidate_rows = []
mismatch = None
evaluator = None
try:
    evaluator = Evaluator('plan', header)
    ps.PassReleaseSelector._guard_curve_at_cap = observe_cap
    for row in trace[1:]:
        kind, data = row['kind'], row['data']
        if kind == 'reset':
            evaluator.reset(data['reason']); resets += 1
            continue
        if kind != 'call': continue
        active_seq = row['seq']; active_measured = data['input']['speed_mps']; slot.clear()
        input_before = digest(data['input'])
        begin = time.perf_counter()
        actual = evaluator(data['input'])
        duration = 1000 * (time.perf_counter() - begin)
        assert input_before == digest(data['input'])
        exact = same(actual, data['output'])
        normal = actual[2].get('normal', {})
        record = dict(seq=active_seq, generation=data['compute']['generation'], worker_sequence=data['compute']['sequence'],
            reference_s=data['input']['reference_s'], exact=exact, baseline_wall_ms_with_instrumentation=duration,
            hook_copy_wall_ms=slot.get('hook_copy_wall_ms', 0), mode=normal.get('selector_mode'),
            sides=normal.get('selector_sides'), reason=normal.get('reason'), original_cap_count=len(slot.get('caps', [])))
        baseline_rows.append(record)
        if not exact:
            mismatch = dict(seq=active_seq, first_difference=first_difference(actual, data['output']))
            write('first_mismatch_actual.json', actual); write('first_mismatch_expected.json', data['output'])
            break
        if not qualifies(slot): continue
        # baseline 및 이후 호출의 실제 상태는 건드리지 않는 복제 owner/args만 쓴다.
        state_before = digest(evaluator.follow)
        snapshot_before = digest(slot['snapshot'])
        owner, args = clone(slot['snapshot'])
        isolated_owner_before = digest(owner)
        begin = time.perf_counter()
        candidate = original_at_cap(owner, *args, target_cap=.1)
        candidate_ms = 1000 * (time.perf_counter() - begin)
        assert state_before == digest(evaluator.follow)
        assert snapshot_before == digest(slot['snapshot'])
        assert isolated_owner_before == digest(owner)
        item = dict(seq=active_seq, reference_s=data['input']['reference_s'], measured_speed_mps=active_measured,
            original_caps=slot['caps'], owner_sides=freeze(owner.sides), guard_anchor=freeze(owner.guard_anchor),
            candidate=freeze(candidate), candidate_wall_ms=candidate_ms,
            accepted=candidate is not None and candidate[0] > 0 and candidate[2].get('selector_guard_candidate', {}).get('accepted') is True,
            additional_calls=1, live_state_unchanged=True, copied_state_unchanged=True,
            source_first_s=args[0].source_first_s, previous_steering_rad=args[5], steering_rad=args[2][1])
        candidate_rows.append(item)
        if active_seq in (895, 998): write(f'sensor_geometry_seq{active_seq}.json', sensor_geometry(owner, args))
        if len(candidate_rows) in (1, 104): print(json.dumps(dict(progress='candidate', seq=active_seq, accepted=item['accepted'])), flush=True)
finally:
    ps.PassReleaseSelector._guard_curve_at_cap = original_at_cap
    if evaluator is not None: evaluator.close()

write('baseline_rows.json', baseline_rows); write('candidate_rows.json', candidate_rows)
module_paths = {name:str(Path(mod.__file__).resolve()) for name, mod in sys.modules.items()
                if name.startswith('arena_autonomy') and getattr(mod, '__file__', None)}
assert all(str(B / 'src/arena_autonomy') in path for path in module_paths.values())
write('module_paths.json', module_paths)
summary = dict(passed=mismatch is None and not changes(), baseline_calls=len(baseline_rows),
    baseline_exact=sum(r['exact'] for r in baseline_rows), first_mismatch=mismatch, reset_count=resets,
    candidate_calls=len(candidate_rows), accepted=sum(r['accepted'] for r in candidate_rows),
    candidate_reasons=dict(collections.Counter(r['candidate'][2].get('selector_reason') for r in candidate_rows)),
    source_changes=changes(), elapsed_wall_s=time.monotonic()-started, python=sys.version, platform=platform.platform(),
    pid=os.getpid(), ros_started=False, devices_opened=False, new_simulation=False, common_source_import=False,
    source_sha256=manifest, scope='진단 복제 후보만 계산; baseline 및 후속 상태에 적용하지 않음')
write('summary.json', summary)
print(json.dumps({k:v for k,v in summary.items() if k!='source_sha256'},ensure_ascii=False),flush=True)
raise SystemExit(0 if summary['passed'] else 2)

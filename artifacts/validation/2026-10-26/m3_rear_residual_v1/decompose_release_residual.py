"""M3 해제 관문의 미확인 가능 영역을 원인·위치별로 분해한다.

저장 판단 입력을 현행 소스로 개루프 재호출하면서 `release_gate`의 결과만 관찰한다. 관찰 래퍼는 원래 함수의
반환값을 그대로 돌려주므로 판단 출력은 바뀌지 않는다(같은 입력의 비관찰 재호출과 전체 출력 대조로 확인).
정답 자세·actor 위치는 읽지 않는다. 폐루프·실시간·안전 판정이 아니다.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
for part in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts', 'tests'):
    sys.path.insert(0, str(ROOT/part))

import numpy as np  # noqa: E402
from arena_autonomy.follow_trace import replay_call, freeze  # noqa: E402
from arena_autonomy.local_follow import FollowConfig  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig  # noqa: E402
import arena_autonomy.m3_execution_trial as mx  # noqa: E402


def brackets(scene, points):
    """depth_support와 같은 양옆 광선 선택. (유효 여부, 아래 광선 상태, 위 광선 상태)."""
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    status = np.array([r.status for r in scene.rays])
    indices = np.array([r.source_index for r in scene.rays])
    valid = np.zeros(len(points), dtype=bool)
    low_status = np.full(len(points), 'none', dtype=object)
    high_status = np.full(len(points), 'none', dtype=object)
    for k in range(0, len(points), 128):
        d = points[k:k+128, None, :]-origins[None, :, :]
        angle = np.arctan2(directions[None, :, 0]*d[:, :, 1]-directions[None, :, 1]*d[:, :, 0],
                           directions[None, :, 0]*d[:, :, 0]+directions[None, :, 1]*d[:, :, 1])
        low = np.where(angle >= -1e-10, np.maximum(angle, 0.), np.inf)
        high = np.where(angle <= 1e-10, np.maximum(-angle, 0.), np.inf)
        lo, hi = np.argmin(low, axis=1), np.argmin(high, axis=1)
        rows = np.arange(len(d))
        valid[k:k+128] = (np.isfinite(low[rows, lo]) & np.isfinite(high[rows, hi])
                          & (low[rows, lo]+high[rows, hi] <= .025) & (np.abs(indices[lo]-indices[hi]) <= 1))
        low_status[k:k+128] = status[lo]
        high_status[k:k+128] = status[hi]
    return valid, low_status, high_status


def classify(record, scene, frame, state, grid, cfg, execution, stop):
    """원래 관문과 같은 식으로 남은 칸을 다시 계산하고 원인·위치를 붙인다(판정에는 쓰지 않음)."""
    rest = record.possible_xy_m
    if rest is None or not len(rest):
        return None
    pad = mx.ray_padding(scene, cfg, execution.history_motion_bound_mps)
    free = mx.depth_support(scene, rest, pad)
    rest = rest[~free]
    if not len(rest):
        return {'possible': 0}
    s, l = frame.project(rest)
    ds = s-state['s0']
    returned = mx._both_sides_returned(scene, rest)
    hits = np.asarray(grid.returns, dtype=float).reshape(-1, 2)
    hit_pad = np.asarray([pad[i] for i, r in enumerate(scene.rays) if r.status == 'return'], dtype=float)
    if len(hits):
        dist = np.sqrt(((rest[:, None, :]-hits[None, :, :])**2).sum(axis=2))
        surface = np.any(dist <= hit_pad[None, :], axis=1)
    else:
        surface = np.zeros(len(rest), dtype=bool)
    blocked = ~(returned & (surface | (ds > cfg.band_ahead_m)))
    valid, lo, hi = brackets(scene, rest)
    masked = (lo == 'masked') | (hi == 'masked')
    unknown = (lo == 'unknown') | (hi == 'unknown')
    cause = np.where(~valid, 'no_bracket', np.where(unknown, 'unknown', np.where(masked, 'mask', 'occluded')))
    behind = rest[:, 0] < -cfg.length_m/2
    sign = 1. if record.side == 'left' else -1.
    gap = cfg.budget.lateral_clearance(True, False, scene.t_ref_s-scene.source_first_s)[0]
    low = sign*state['d0']-state['half_span_m']
    high = max(sign*state['d0'], 0.)+state['half_span_m']+gap
    corridor = (ds >= -cfg.length_m/2) & (ds <= cfg.length_m/2+stop) & (sign*l >= low) & (sign*l <= high)
    out = {'possible': int(len(rest)), 'blocked': int(blocked.sum()), 'corridor': int(corridor.sum())}
    for name, mask in (('blocked', blocked), ('corridor', corridor)):
        counts = Counter()
        for c, b in zip(cause[mask], behind[mask]):
            counts[f"{c}_{'behind' if b else 'not_behind'}"] += 1
        out[f'{name}_by_cause'] = dict(counts)
    if np.any(blocked):
        x = rest[blocked]
        out['blocked_x_max_m'] = float(np.max(x[:, 0]))
        out['blocked_not_mask_or_behind'] = int(np.sum(blocked & ~((cause == 'mask') & behind)))
        out['blocked_not_mask'] = int(np.sum(blocked & (cause != 'mask')))
    return out


def run(path, instrument):
    raw = path.read_bytes()
    events = [json.loads(x) for x in raw.decode('utf8').splitlines()]
    head = events[0]['data']
    controller = mx.M3ExecutionFollow(FollowConfig(**head['follow_config']),
        mx.ExecutionConfig(**head['execution_config']), allow_simulation_trial=True)
    motion = MotionConfig(**head['motion_config'])
    original = mx.release_gate
    checks, outputs, steps = [], [], []
    current = {}

    def observe(record, scene, history, frame, state, grid, cfg, execution, stop):
        gate = original(record, scene, history, frame, state, grid, cfg, execution, stop)
        item = {k: gate.get(k) for k in ('side', 'ok', 'release', 'reason', 'region_cells', 'possible_cells',
                                          'blocked_cells', 'corridor_cells', 'grow_m', 'body_cells')}
        item.update(t=scene.t_ref_s, detected_at=record.detected_at_s, last_confirmed_at=record.last_confirmed_at_s,
                    first_step=record.possible_xy_m is None)
        if gate['record'] is not None and gate['ok']:
            item['classes'] = classify(gate['record'], scene, frame, state, grid, cfg, execution, stop)
        current.setdefault('checks', []).append(item)
        return gate

    if instrument:
        mx.release_gate = observe
    try:
        for event in events[1:-1]:
            if event['kind'] == 'reset':
                controller.reset()
                continue
            current.clear()
            result = replay_call(controller, event['data']['input'], motion)
            frozen = freeze(result)
            outputs.append(hashlib.sha256(json.dumps(frozen, sort_keys=True).encode()).hexdigest())
            lateral = result[2].get('m3_lateral') or {}
            gate = lateral.get('release_gate') or {}
            threats = lateral.get('threats') or {}
            steps.append({'t': event['data']['input']['reference_s'], 'mode': lateral.get('mode'),
                          'present': [k for k, v in threats.items() if v.get('present')],
                          'gate_reason': gate.get('reason'), 'memory_records': gate.get('memory_records'),
                          'memory_blocked': gate.get('memory_blocked'), 'speed': result[0],
                          'released': bool(lateral.get('released_to_baseline')),
                          'checks': current.get('checks', [])})
    finally:
        mx.release_gate = original
    unchanged = hashlib.sha256(path.read_bytes()).hexdigest() == hashlib.sha256(raw).hexdigest()
    return steps, outputs, unchanged, hashlib.sha256(raw).hexdigest()


def summarize(steps):
    gated = [s for s in steps if s['checks']]
    first_active = next((s['t'] for s in steps if s['mode'] not in (None, 'BASELINE', 'OFF')), None)
    last_present = max((s['t'] for s in steps if s['present']), default=None)
    blocked_first = next((s['t'] for s in steps if s['memory_blocked']), None)
    after = [s for s in gated if last_present is None or s['t'] > last_present]
    cause_totals = Counter()
    for s in gated:
        for c in s['checks']:
            for k, v in ((c.get('classes') or {}).get('blocked_by_cause') or {}).items():
                cause_totals[k] += v
    per_step = []
    for s in gated:
        rows = [c for c in s['checks'] if c.get('classes')]
        per_step.append({'t': s['t'], 'present': s['present'], 'records_checked': len(s['checks']),
            'blocked': sum(c['classes'].get('blocked', 0) for c in rows),
            'blocked_not_mask_or_behind': sum(c['classes'].get('blocked_not_mask_or_behind', 0) for c in rows),
            'blocked_not_mask': sum(c['classes'].get('blocked_not_mask', 0) for c in rows),
            'corridor': sum(c['classes'].get('corridor', 0) for c in rows),
            'reasons': dict(Counter(c['reason'] for c in s['checks'])),
            'max_detection_age_s': max((s['t']-(c['detected_at'] if c['detected_at'] is not None else c['t'])
                                         for c in s['checks']), default=None)})
    return {'decisions': len(steps), 'gate_steps': len(gated), 'first_active_t': first_active,
            'last_threat_present_t': last_present, 'memory_blocked_first_t': blocked_first,
            'released_at': [s['t'] for s in steps if s['released']],
            'blocked_cause_totals': dict(cause_totals), 'per_step': per_step,
            'steps_after_last_present': len(after)}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('inputs', nargs='+', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    source = Path(mx.__file__)
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    result = {'scope': 'stored-input open-loop replay with current source; observation only, not closed loop',
              'm3_source_sha256': digest, 'inputs': {}}
    for path in args.inputs:
        plain_steps, plain, ok1, sha = run(path, False)
        steps, observed, ok2, _ = run(path, True)
        name = path.parent.parent.name
        entry = summarize(steps)
        entry.update(sha256=sha, input_unchanged=ok1 and ok2,
                     instrumented_output_differences=sum(a != b for a, b in zip(plain, observed)),
                     outputs=len(observed))
        result['inputs'][name] = entry
        print(json.dumps({name: {k: v for k, v in entry.items() if k != 'per_step'}}, ensure_ascii=False), flush=True)
    result['source_unchanged'] = hashlib.sha256(source.read_bytes()).hexdigest() == digest
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
    assert result['source_unchanged']
    assert all(v['input_unchanged'] and v['instrumented_output_differences'] == 0 for v in result['inputs'].values())

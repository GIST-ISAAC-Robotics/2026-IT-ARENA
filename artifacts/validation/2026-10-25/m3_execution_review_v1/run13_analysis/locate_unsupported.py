"""run13 저장 판단을 같은 소스로 재호출해, 명령/계획 관측 거절 질의점의 위치와 원인을 기록한다.

제어 출력은 원래 기록과 전부 같아야 한다(불일치 시 실패). 질의점 분류는 설명용이며 판정·출력은 바꾸지 않는다.
좌표는 현재 차체 기준(차체 중심 원점, +x 전방, +y 왼쪽)이다.
"""
import argparse
from collections import Counter, defaultdict
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import numpy as np

import arena_autonomy.m3_execution_trial as m3
from arena_autonomy.follow_trace import replay_call, freeze, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, FollowConfig as _FC
from arena_autonomy.observation_support import depth_support, source_valid
from arena_autonomy.observation_lateral import reachable_from_unknown

CONTEXT = {'stage': None, 'candidate': None, 'seq': None}
RECORDS = []
ORIGINAL_SUPPORT = m3.support
ORIGINAL_ARC = m3.arc_checks
ORIGINAL_PLAN = m3.plan_check
ORIGINAL_EVAL = m3.M3ExecutionFollow._evaluate_command


def classify(scene, frames, queries, cfg, execution):
    """원래 support와 같은 순서로 최종 미지지 질의와 원인을 계산한다."""
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    pad = m3.ray_padding(scene, cfg, execution.history_motion_bound_mps)
    ok = depth_support(scene, q, pad) if len(q) else np.zeros(0, dtype=bool)
    cause = np.array(['current'] * len(q), dtype=object)
    cause[~ok] = 'not_masked_only'
    if execution.masked_history_support and np.any(~ok) and frames:
        missing = np.flatnonzero(~ok)
        masked = m3.masked_only(scene, q[missing], pad)
        cause[missing[masked]] = 'masked_no_history_view'
        remaining = missing[masked]
        reach_config = _FC(length_m=cfg.length_m, width_m=cfg.width_m,
                           object_motion_bound_mps=execution.history_motion_bound_mps)
        for old, (dx, dy, yaw) in frames:
            if not len(remaining):
                break
            if (not source_valid(old) or old.source_epoch != scene.source_epoch
                    or old.source_first_s >= scene.source_first_s):
                continue
            c, sn = math.cos(yaw), math.sin(yaw)
            local = (q[remaining]-[dx, dy])@np.array([[c, -sn], [sn, c]])
            old_pad = np.array([.025+cfg.budget.range_bias_m+execution.history_motion_bound_mps*(
                scene.t_ref_s-r.acquired_at_s) for r in old.rays])
            accepted = depth_support(old, local, old_pad)
            possible = np.flatnonzero(accepted)
            if len(possible):
                reached = reachable_from_unknown(replace(old, t_ref_s=scene.t_ref_s), local[possible], reach_config)
                cause[remaining[possible[reached]]] = 'masked_history_reachable'
                accepted[possible[reached]] = False
            ok[remaining[accepted]] = True
            cause[remaining[accepted]] = 'history'
            remaining = remaining[~accepted]
    elif np.any(~ok):
        missing = np.flatnonzero(~ok)
        masked = m3.masked_only(scene, q[missing], pad)
        cause[missing[masked]] = 'masked_no_history_frames'
    return q, ok, cause


def wrapped_support(scene, frames, queries, cfg, execution):
    detail = ORIGINAL_SUPPORT(scene, frames, queries, cfg, execution)
    if detail['unsupported']:
        q, ok, cause = classify(scene, frames, queries, cfg, execution)
        assert int(np.sum(~ok)) == detail['unsupported'], 'classification drift'
        origins = np.asarray([r.ray_origin_xy_m for r in scene.rays])
        sensor = origins.mean(axis=0)
        lx, wy = cfg.length_m/2, cfg.width_m/2
        rows = []
        for (x, y), c in zip(q[~ok], cause[~ok]):
            outside_lat = abs(y)-wy
            outside_long = abs(x)-lx
            bearing = math.degrees(math.atan2(y-sensor[1], x-sensor[0]))
            rows.append({'x': float(x), 'y': float(y), 'outside_lateral_m': float(outside_lat),
                         'outside_longitudinal_m': float(outside_long), 'bearing_from_sensor_deg': bearing,
                         'cause': str(c)})
        RECORDS.append({'seq': CONTEXT['seq'], 'stage': CONTEXT['stage'], 'candidate': CONTEXT['candidate'],
                        'sensor_xy_mean': sensor.tolist(), 'unsupported': rows})
    return detail


def wrapped_arc(*args, **kwargs):
    previous = CONTEXT['stage']
    CONTEXT['stage'] = 'command_envelope'
    try:
        return ORIGINAL_ARC(*args, **kwargs)
    finally:
        CONTEXT['stage'] = previous


def wrapped_plan(scene, frames, candidate, *args, **kwargs):
    previous = CONTEXT['stage']
    CONTEXT['stage'] = 'plan:'+str(candidate.get('name', '?'))
    try:
        return ORIGINAL_PLAN(scene, frames, candidate, *args, **kwargs)
    finally:
        CONTEXT['stage'] = previous


def wrapped_eval(self, name, *args, **kwargs):
    previous = CONTEXT['candidate']
    CONTEXT['candidate'] = name
    try:
        return ORIGINAL_EVAL(self, name, *args, **kwargs)
    finally:
        CONTEXT['candidate'] = previous


def main():
    p = argparse.ArgumentParser()
    p.add_argument('input', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise ValueError('new output required')
    raw = args.input.read_bytes()
    events = [json.loads(line) for line in raw.decode().splitlines()]
    head = events[0]['data']
    assert events[0]['kind'] == 'header' and head['controller_mode'] == 'm3_execution_trial'
    motion, config = MotionConfig(**head['motion_config']), FollowConfig(**head['follow_config'])
    execution = m3.ExecutionConfig(**head['execution_config'])
    m3.support, m3.arc_checks, m3.plan_check = wrapped_support, wrapped_arc, wrapped_plan
    m3.M3ExecutionFollow._evaluate_command = wrapped_eval
    follow = m3.M3ExecutionFollow(config, execution, allow_simulation_trial=True)
    exact = calls = 0
    outputs = {}
    for event in events[1:-1]:
        if event['kind'] == 'reset':
            follow.reset()
            continue
        CONTEXT.update(seq=event['seq'], stage=None, candidate=None)
        out = freeze(replay_call(follow, event['data']['input'], motion))
        calls += 1
        exact += equal_output(out, event['data']['output'])
        outputs[event['seq']] = (event['data']['input']['reference_s'], out[0], out[2]['reason'],
                                 out[2].get('m3_lateral', {}).get('mode'), event['data']['input']['speed_mps'],
                                 event['data']['input']['previous_steering_rad'])
    by_seq = defaultdict(list)
    for r in RECORDS:
        by_seq[r['seq']].append(r)
    # 전 후보 명령 거절(HOLD_STOP_TRIAL + m3_command_unobserved) 판단의 명령 외곽 미지지 질의만 요약한다.
    focus = [s for s, o in outputs.items() if o[2] == 'm3_command_unobserved']
    summary = Counter()
    max_outside = []
    moving_focus = []
    for s in focus:
        pts = [u for r in by_seq[s] if r['stage'] == 'command_envelope' for u in r['unsupported']]
        for u in pts:
            summary[u['cause']] += 1
        if pts:
            worst = max(max(u['outside_lateral_m'], u['outside_longitudinal_m']) for u in pts)
            max_outside.append(worst)
            if abs(outputs[s][4]) > 1e-6:
                moving_focus.append(s)
    def pct(values, q):
        return float(np.percentile(values, q)) if values else None
    bearings = [abs(180-abs(u['bearing_from_sensor_deg'])) for s in focus for r in by_seq[s]
                if r['stage'] == 'command_envelope' for u in r['unsupported']]
    result = {'scope': 'stored-input re-call with identical outputs; query location/cause explanation only',
              'input_sha256': hashlib.sha256(raw).hexdigest(), 'calls': calls, 'exact_output_matches': exact,
              'focus_calls_m3_command_unobserved': len(focus), 'focus_calls_measured_moving': len(moving_focus),
              'command_envelope_unsupported_by_cause': dict(summary),
              'focus_worst_outside_body_m': {'p50': pct(max_outside, 50), 'p95': pct(max_outside, 95),
                                             'max': max(max_outside) if max_outside else None},
              'command_envelope_unsupported_angle_from_rear_deg': {'min': min(bearings) if bearings else None,
                                                                   'p50': pct(bearings, 50), 'max': max(bearings) if bearings else None},
              'examples': {str(s): {'t': outputs[s][0], 'measured_speed': outputs[s][4], 'previous_steering': outputs[s][5],
                                    'records': by_seq[s]} for s in focus[:6] + moving_focus[:6]},
              'per_call': {str(s): {'t': outputs[s][0], 'speed_out': outputs[s][1], 'reason': outputs[s][2],
                                    'mode': outputs[s][3], 'measured_speed': outputs[s][4],
                                    'previous_steering': outputs[s][5],
                                    'records': [{'stage': r['stage'], 'candidate': r['candidate'],
                                                 'n': len(r['unsupported']),
                                                 'causes': dict(Counter(u['cause'] for u in r['unsupported'])),
                                                 'max_outside_m': max(max(u['outside_lateral_m'], u['outside_longitudinal_m'])
                                                                      for u in r['unsupported'])} for r in by_seq[s]]}
                           for s in sorted(outputs) if by_seq[s]}}
    result['input_unchanged'] = hashlib.sha256(args.input.read_bytes()).hexdigest() == result['input_sha256']
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('examples', 'per_call')}, ensure_ascii=False, indent=1))
    assert result['input_unchanged'] and exact == calls


if __name__ == '__main__':
    main()

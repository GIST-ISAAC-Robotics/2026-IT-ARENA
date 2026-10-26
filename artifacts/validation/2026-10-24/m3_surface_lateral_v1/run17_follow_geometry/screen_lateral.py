"""측방 의도 제한기 v1의 저장 입력 개루프 선별. 주행·src 운영 연결 없음.

후보 열은 판단 기록(센서 입력·당시 기본 경로)만 써서 정답 파일을 연 순간 예외가 나도록 막은 상태에서
계산한다. 정답 자세는 그 뒤 eval_* 열에서만 쓴다. 저장 궤적은 원래 정책(G1/R0+)의 것이므로 모든
결과는 개루프 판정이며, 후보 경로를 실제로 따랐을 때의 자세·센서열·간격이 아니다.
"""
import argparse
import builtins
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p/'src/arena_autonomy').is_dir())
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'scripts')]
from arena_autonomy.follow_trace import restore_call, thaw  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable  # noqa: E402
from arena_autonomy.local_follow import FollowConfig, pursue_path  # noqa: E402
from arena_autonomy.local_path import braking_speed, swept_limit  # noqa: E402
from arena_autonomy.occupancy_timing import per_ray_padding  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402
import arena_autonomy.lateral_intent as li  # noqa: E402

RECORDS = {
    'e3_onroad_g1': 'artifacts/validation/2026-10-24/e3_onroad_v1/g1',
    'e3_onroad_r0plus': 'artifacts/validation/2026-10-24/e3_onroad_v1/r0plus',
    'e3_stopped_g1': 'artifacts/validation/2026-10-24/e3_harness_v1/run',
    'e3_stop_restart_g1': 'artifacts/validation/2026-10-24/e3_stop_restart_v1/run',
    'e3_stopped_r0plus': 'artifacts/validation/2026-10-24/r0plus_core_v1/stopped_center',
    'solo_g1_20261016': 'artifacts/validation/2026-10-16/g1_corridor_solo_v1/run',
}
STOPPED = ('e3_stopped_g1', 'e3_stopped_r0plus')
TRUTH_NAMES = ('ego_poses', 'additional_actors', 'actor_poses', 'trajectory', 'runtime_world')
_PATH_OPEN = Path.open


def variants():
    base = dict(enabled=True)
    return {
        'road_only_hard': li.LateralIntentConfig(**base, surface_policy='road_only'),
        'grass_hard': li.LateralIntentConfig(**base, surface_policy='two_wheel_grass'),
    }


def sensitivity():
    base = dict(enabled=True, surface_policy='two_wheel_grass')
    return {
        'grass_hard_hidden1.0': li.LateralIntentConfig(**base, hidden_relative_speed_mps=1.0),
        'grass_hard_hidden2.0': li.LateralIntentConfig(**base, hidden_relative_speed_mps=2.0),
        'grass_hard_hidden0.2': li.LateralIntentConfig(**base, hidden_relative_speed_mps=.2),
        'grass_hard_commit0.3': li.LateralIntentConfig(**base, commit_horizon_s=.3),
    }


class TruthBlocked:
    def __enter__(self):
        self.original = builtins.open
        original = self.original

        def guarded(file, *args, **kwargs):
            if any(name in str(file) for name in TRUTH_NAMES):
                raise RuntimeError(f'truth file opened during candidate computation: {file}')
            return original(file, *args, **kwargs)
        builtins.open = guarded
        Path.open = lambda self, *a, **k: guarded(str(self), *a, **k)
        return self

    def __exit__(self, *exc):
        builtins.open = self.original
        Path.open = _PATH_OPEN
        return False


def load_inputs(base):
    calls, motion, follow = [], None, None
    with (ROOT/base/'follow_decisions.jsonl').open() as stream:
        for raw in stream:
            event = json.loads(raw)
            if event['kind'] == 'header':
                motion = MotionConfig(**event['data']['motion_config'])
                follow = FollowConfig(**event['data']['follow_config'])
            elif event['kind'] in ('call', 'reset'):
                calls.append(event)
    return motion, follow, calls


def existing_checks(path, points, margin, speed, previous_steering, age, follow_cfg, control_dt=.02,
                    path_age=None):
    """G1 `_baseline_command`의 경로 이후 계산과 같은 함수·순서(HOLD 이력 제외)."""
    try:
        cap, desired, remaining = pursue_path(path, speed, follow_cfg, age if path_age is None else path_age)
    except MotionUnavailable as error:
        return {'status': str(error)}
    steering = float(np.clip(desired, previous_steering-3*control_dt, previous_steering+3*control_dt))
    clearance = min(swept_limit(points, a, follow_cfg.wheelbase_m, horizon=2.5, margin=margin,
                                length=follow_cfg.length_m, width=follow_cfg.width_m)
                    for a in (steering, previous_steering))
    available = max(0., clearance-follow_cfg.standstill_gap_m)
    target = min(cap, braking_speed(available, age), available/follow_cfg.headway_s)
    return {'status': 'ok', 'steering_rad': steering, 'swept_clearance_m': clearance, 'target_mps': target,
            'gap_hold': clearance <= follow_cfg.hold_clearance_m}


def compact_check(check):
    out = {k: check.get(k) for k in ('d_target_m', 'verified', 'verified_except_hidden', 'reasons', 'problems',
                                     'new_cells', 'new_cells_unobserved', 'hidden_reach_cells', 'residual_risk')}
    surf = check.get('surface') or {}
    out['surface'] = {k: surf.get(k) for k in ('ok', 'grass_side', 'reason', 'max_body_excess_m')}
    lat = check.get('lateral_clearance') or {}
    out['lateral_min_margin_m'] = lat.get('min_margin_m')
    return out


def run_record(name, cfg, motion, follow, calls, *, window=None, bypass=False):
    limiter = li.LateralIntentLimiter(cfg)
    rows, last_source, chain = [], None, None
    for event in calls:
        if event['kind'] == 'reset':
            limiter.reset()
            last_source = None
            continue
        t = event['data']['input']['reference_s']
        if t is None or (window and not window[0] <= t <= window[1]):
            continue
        try:
            data, scan, history = restore_call(thaw(event['data']['input']), motion)
            scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                 epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        except (MotionUnavailable, ValueError) as error:
            limiter.reset()
            rows.append({'t': t, 'status': 'input_error', 'error': str(error)})
            continue
        output = event['data']['output']
        detail = output[2]
        path = detail.get('path_points')
        path = None if path is None else np.asarray(path, dtype=float)
        fresh = last_source is None or scene.source_first_s > last_source
        last_source = scene.source_first_s
        started = time.perf_counter()
        decision = limiter.step(scene, path, data['speed_mps'], fresh=fresh,
                                previous_steering=data['previous_steering_rad'])
        elapsed = time.perf_counter()-started
        row = {'t': t, 'status': 'ok', 'recorded_reason': detail.get('reason'), 'recorded_speed': output[0],
               'recorded_steering': output[1], 'measured_speed': data['speed_mps'], 'mode': decision['mode'],
               'selected': decision.get('selected'), 'reasons': decision.get('reasons'), 'step_ms': elapsed*1e3}
        if path is not None and decision['mode'] != 'NO_BASE_PATH':
            st = decision['state']
            row.update(d0=st['d0'], psi=st['psi'],
                       threats={k: {'present': v['present'], 'points': v['points'],
                                    'gap_m': (v.get('nearest') or {}).get('gap_m'),
                                    'visible_gap_m': (v.get('nearest') or {}).get('visible_gap_m'),
                                    'near_edge_source': (v.get('nearest') or {}).get('near_edge_source')}
                                for k, v in decision['threats'].items()},
                       checks={k: compact_check(v) for k, v in decision['checks'].items()},
                       edge_verified={k: decision['corridor'][k]['edge_verified'] for k in ('left', 'right')},
                       grass_time_s=decision['grass_time_s'])
            if decision.get('braking_steering'):
                row['braking_steering'] = decision['braking_steering']
            if decision.get('path') is not None:
                points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
                margins = per_ray_padding(scene, .025+follow.measurement_padding_m, follow.object_motion_bound_mps)
                age = data['reference_s']-scene.source_first_s
                age_eff = max(age, detail.get('boundary_age_s') or 0.)
                row['downstream'] = existing_checks(decision['path'], points, margins, data['speed_mps'],
                                                    data['previous_steering_rad'], age, follow, data['dt_s'],
                                                    path_age=age_eff)
            if bypass and decision['mode'] != 'NO_BASE_PATH':
                row['bypass'] = bypass_screen(scene, path, data['speed_mps'])
        rows.append(row)
    return rows


def bypass_screen(scene, path, speed):
    """앞 물체를 좌/우로 지나는 후보의 정책별 검증(정지 가정 유무). 주행 결정이 아니라 공간 예산 비교."""
    out = {}
    for policy in ('road_only', 'two_wheel_grass'):
        cfg = li.LateralIntentConfig(enabled=True, surface_policy=policy)
        frame = li.PathFrame(path)
        context = li.build_context(scene, frame, speed, cfg)
        hold = li.offset_candidate(frame, context['state'], context['state']['d0'], cfg, 'hold')
        for side in ('left', 'right'):
            for stationary in (True, False):
                cand = li.bypass_candidate(frame, context['state'], context['grid'].returns, side, cfg,
                                           stationary=stationary)
                key = f'{policy}:{side}:{"stationary" if stationary else "moving"}'
                if cand is None:
                    out[key] = {'status': 'no_in_path_object'}
                    continue
                check = li.verify_candidate(cand, hold, context, cfg, discretionary=True)
                out[key] = {'d_target_m': cand['d_target_m'], 'obstacle_l_range_m': cand['obstacle_l_range_m'],
                            'clearance_m': cand['bypass_clearance_m'], **compact_check(check)}
    return out


# ------------------------------------------------------------------ 평가 전용(정답)
def attach_truth(base, rows):
    ego_path = ROOT/base/'ego_poses.jsonl'
    if not ego_path.exists():
        return
    ego = [json.loads(line) for line in ego_path.read_text(encoding='utf-8').splitlines() if line.strip()]
    et = [r[0] for r in ego]
    actors = {}
    lead = ROOT/base/'actor_poses.jsonl'
    if lead.exists():
        actors['lead'] = [json.loads(line) for line in lead.read_text(encoding='utf-8').splitlines() if line.strip()]
    extra = ROOT/base/'additional_actors.json'
    if extra.exists():
        for key, poses in json.loads(extra.read_text(encoding='utf-8'))['poses'].items():
            actors['side' if 'side' in key else key] = poses
    times = {k: [p[0] for p in v] for k, v in actors.items()}
    for row in rows:
        i = bisect_right(et, row['t'])-1
        if i < 0:
            continue
        _, x, y, yaw = ego[i][:4]
        c, s = math.cos(yaw), math.sin(yaw)
        row['eval'] = {}
        for key, poses in actors.items():
            j = bisect_right(times[key], row['t'])-1
            if j < 0:
                continue
            _, ax, ay, ayaw = poses[j][:4]
            rel = [c*(ax-x)+s*(ay-y), -s*(ax-x)+c*(ay-y)]
            # 평행 가정의 측방 차체 간격(종방향으로 겹칠 때만 의미): |dy| - 0.15
            row['eval'][key] = {'rel_xy': rel, 'side_by_side': abs(rel[0]) < .20,
                                'lateral_body_gap_m': abs(rel[1])-.15}


# ------------------------------------------------------------------ 요약
def summarize(rows):
    ok = [r for r in rows if r.get('status') == 'ok']
    modes, reasons, selected = {}, {}, {}
    for r in ok:
        modes[r['mode']] = modes.get(r['mode'], 0)+1
        selected[str(r.get('selected'))] = selected.get(str(r.get('selected')), 0)+1
        for reason in r.get('reasons') or []:
            reasons[reason] = reasons.get(reason, 0)+1
    first = {}
    for r in ok:
        first.setdefault(r['mode'], round(r['t'], 3))
    threat_rows = [r for r in ok if any(v['present'] for v in (r.get('threats') or {}).values())]
    toward = 0
    for r in ok:
        sel = r.get('selected')
        if sel and r.get('threats') and sel in r.get('checks', {}):
            delta = r['checks'][sel]['d_target_m']-r['d0']
            for side, sign in (('left', 1), ('right', -1)):
                if r['threats'][side]['present'] and sign*delta > .005:
                    toward += 1
    down = [r['downstream'] for r in ok if r.get('downstream', {}).get('status') == 'ok']
    yields = [r for r in ok if r['mode'] == 'YIELD']
    steps = [r['step_ms'] for r in ok]
    check_reasons = {}
    for r in ok:
        for key, check in (r.get('checks') or {}).items():
            for reason in check.get('reasons') or []:
                check_reasons[f'{key}:{reason}'] = check_reasons.get(f'{key}:{reason}', 0)+1
    return {'decisions': len(rows), 'ok': len(ok), 'input_errors': len(rows)-len(ok), 'modes': modes,
            'first_time_by_mode': first, 'selected': selected, 'decision_reasons': reasons,
            'candidate_reasons': check_reasons, 'threat_decisions': len(threat_rows),
            'selected_toward_present_threat': toward, 'yield_decisions': len(yields),
            'yield_targets_m': [round(r['checks']['yield']['d_target_m'], 4) for r in yields][:12],
            'downstream_checked': len(down), 'downstream_gap_hold': sum(d['gap_hold'] for d in down),
            'downstream_positive': sum(d['target_mps'] > 0 and not d['gap_hold'] for d in down),
            'step_ms_p50_p95_max': np.percentile(steps, [50, 95, 100]).round(2).tolist() if steps else None}


def truth_summary(rows):
    """평가 전용: 옆 actor가 측후방 창(뒤 0.6~앞 0.35 m, 옆)에 있었던 판단 중 위협 표시 비율."""
    inside, flagged, false_side = 0, 0, 0
    for r in rows:
        side = (r.get('eval') or {}).get('side')
        if not side or r.get('status') != 'ok' or not r.get('threats'):
            continue
        x, y = side['rel_xy']
        in_window = -.6-.1 <= x <= .35+.1 and abs(y) < .55
        if in_window:
            inside += 1
            key = 'left' if y > 0 else 'right'
            flagged += r['threats'][key]['present']
            false_side += r['threats']['right' if key == 'left' else 'left']['present']
    return {'side_actor_in_window_decisions': inside, 'flagged_on_actor_side': flagged,
            'flagged_on_opposite_side': false_side}


def compact_rows(rows, window):
    keep = []
    for r in rows:
        if r.get('status') != 'ok' or not window[0] <= r['t'] <= window[1]:
            continue
        item = {k: r.get(k) for k in ('t', 'recorded_reason', 'recorded_speed', 'mode', 'selected', 'reasons', 'd0')}
        item['threats'] = {k: (v['present'], v['gap_m'], v['visible_gap_m']) for k, v in (r.get('threats') or {}).items()}
        item['targets'] = {k: (round(v['d_target_m'], 4), v['verified'], v['reasons']) for k, v in (r.get('checks') or {}).items()}
        if 'downstream' in r:
            item['downstream'] = {k: r['downstream'].get(k) for k in ('status', 'target_mps', 'gap_hold', 'swept_clearance_m')}
        if 'eval' in r:
            item['eval'] = {k: [round(v['rel_xy'][0], 3), round(v['rel_xy'][1], 3), round(v['lateral_body_gap_m'], 3)]
                            for k, v in r['eval'].items()}
        keep.append(item)
    return keep


def bypass_summary(rows):
    out = {}
    for r in rows:
        if r.get('status') != 'ok' or 'bypass' not in r or r.get('recorded_speed', 1) > 0:
            continue
        for key, value in r['bypass'].items():
            entry = out.setdefault(key, {'evaluated': 0, 'verified': 0, 'no_object': 0, 'reasons': {}})
            if value.get('status') == 'no_in_path_object':
                entry['no_object'] += 1
                continue
            entry['evaluated'] += 1
            entry['verified'] += bool(value['verified'])
            for reason in value.get('reasons') or []:
                entry['reasons'][reason] = entry['reasons'].get(reason, 0)+1
            entry.setdefault('d_target_m_examples', []).append(round(value['d_target_m'], 4)) if len(
                entry.get('d_target_m_examples', [])) < 5 else None
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--records', nargs='*', default=list(RECORDS))
    parser.add_argument('--no-sensitivity', action='store_true')
    parser.add_argument('--no-bypass', action='store_true', help='정지차 우회는 별도 설계 전에는 대조하지 않는다')
    args = parser.parse_args()
    out = {'scope': 'open-loop screening of lateral intent limiter v1 on stored inputs; candidate columns computed '
                    'with truth files blocked; eval only after; not closed-loop, driving or safety evidence',
           'records': {}, 'sources': {}}
    for name in args.records:
        base = RECORDS[name]
        motion, follow, calls = load_inputs(base)
        entry = {}
        configs = dict(variants())
        if name in ('e3_onroad_g1', 'e3_onroad_r0plus') and not args.no_sensitivity:
            configs.update(sensitivity())
        for key, cfg in configs.items():
            with TruthBlocked():
                rows = run_record(name, cfg, motion, follow, calls,
                                  bypass=(not args.no_bypass and name in STOPPED and key == 'grass_hard'))
            attach_truth(base, rows)
            result = {'summary': summarize(rows), 'truth': truth_summary(rows)}
            if name in STOPPED and key == 'grass_hard':
                result['bypass_while_recorded_stopped'] = bypass_summary(rows)
            if key in ('grass_hard', 'road_only_hard', 'grass_soft_quiet') and name.startswith('e3_onroad'):
                windows = [(3.8, 7.0), (10.2, 15.2)] if name == 'e3_onroad_r0plus' else [(3.8, 7.0)]
                result['timeline'] = [compact_rows(rows, w) for w in windows]
            entry[key] = result
            print(name, key, json.dumps(result['summary']['modes']), json.dumps(result['truth']), flush=True)
        out['records'][name] = entry
        out['sources'][f'{base}/follow_decisions.jsonl'] = hashlib.sha256(
            (ROOT/base/'follow_decisions.jsonl').read_bytes()).hexdigest()
    for f in ('src/arena_autonomy/arena_autonomy/lateral_intent.py', 'src/arena_autonomy/arena_autonomy/local_follow.py',
              'src/arena_autonomy/arena_autonomy/local_path.py', 'src/arena_autonomy/arena_autonomy/local_scene.py',
              'src/arena_autonomy/arena_autonomy/scene_motion.py', 'src/arena_autonomy/arena_autonomy/follow_trace.py'):
        out['sources'][f] = hashlib.sha256((ROOT/f).read_bytes()).hexdigest()
    out['sources']['screen_lateral.py'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(out, indent=1, ensure_ascii=False, default=float), encoding='utf-8', newline='\n')


if __name__ == '__main__':
    main()

"""A2/P1 고정 판 1회 대조: 저장 3기록 + G1 창 반례. 주행·src 변경 없음.

후보 열은 candidate.py만 계산하며 정답 파일을 열지 않는다(열면 예외). 정답 자세는
evaluation_* 열에서만 쓴다. 저장 궤적은 원래 정책의 것이므로 개루프 판정이다.
"""
import builtins
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p/'src/arena_autonomy').is_dir())
sys.path[:0] = [str(HERE), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import candidate as c  # noqa: E402
from arena_autonomy.follow_trace import restore_call, thaw  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable  # noqa: E402
from arena_autonomy.local_follow import FollowConfig  # noqa: E402
from arena_autonomy.occupancy_timing import per_ray_padding  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402

CFG = c.A2Config()
TRACES = {'e3_onroad_g1': 'artifacts/validation/2026-10-24/e3_onroad_v1/g1',
          'e3_onroad_r0plus': 'artifacts/validation/2026-10-24/e3_onroad_v1/r0plus',
          'solo_g1_20261016': 'artifacts/validation/2026-10-16/g1_corridor_solo_v1/run'}
G1_WINDOW = (3.8, 7.0)
TRUTH_NAMES = ('ego_poses', 'additional_actors')


class TruthBlocked:
    """후보 계산 중 정답 파일 열기를 막는다."""

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


_PATH_OPEN = Path.open


def load_inputs(base):
    """판단 기록(센서 입력·당시 출력)만 읽는다."""
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


def scene_inputs(event, motion, follow):
    data, scan, history = restore_call(thaw(event['data']['input']), motion)
    scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                         epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
    points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
    margins = per_ray_padding(scene, .025+follow.measurement_padding_m, follow.object_motion_bound_mps)
    bearings = np.array([math.remainder(scan.angle_min+r.source_index*scan.angle_increment, math.tau)
                         for r in scene.rays])
    status = np.array([r.status for r in scene.rays])
    return_bearings = bearings[status == 'return']
    ages = np.array([scene.t_ref_s-r.acquired_at_s for r in scene.rays if r.status == 'return'])
    return data, scene, points, margins, bearings, status, return_bearings, ages


def candidate_pass(name, motion, follow, calls, window=None, variant=None):
    """후보 열 계산. variant: None | 'missing_all' | 'missing_after_trigger' | 'mirror_static' | 'mirror_time'."""
    machine = c.A2Machine(CFG)
    rows, last_source = [], None
    triggered_at = trigger_offset = None
    p1_prev = None   # P1 활성 중에는 후보 자신의 직전 조향을 잇는다(원래 G1 조향의 변화율 결합 제거)
    for event in calls:
        if event['kind'] == 'reset':
            machine.reset()
            last_source = None
            continue
        t = event['data']['input']['reference_s']
        if t is None or (window and not window[0] <= t <= window[1]):
            continue
        try:
            data, scene, points, margins, bearings, status, rb, ages = scene_inputs(event, motion, follow)
        except (MotionUnavailable, ValueError):
            rows.append({'t': t, 'status': 'input_error'})
            continue
        output = event['data']['output']
        detail = output[2]
        path = detail.get('path_points')
        path = None if path is None else np.asarray(path, dtype=float)
        if variant in ('missing_all', 'missing_after_trigger') and (variant == 'missing_all' or
                                                                    (triggered_at is not None and t > triggered_at)):
            drop = (np.degrees(rb) >= 90) & (np.degrees(rb) <= 150)
            points, margins, ages = points[~drop], margins[~drop], ages[~drop]
            deg = np.degrees(bearings)
            status = status.astype('<U8')   # run1 결함: '<U6' 배열에 'unknown'을 넣으면 'unknow'로 잘림
            status[(status == 'return') & (deg >= 90) & (deg <= 150)] = 'unknown'
            if np.any(drop) and not np.any(status == 'unknown'):
                raise AssertionError('missing injection did not produce unknown rays')
        if variant in ('mirror_static', 'mirror_time'):
            points = points*[1., -1.]
            path = None if path is None else path*[1., -1.]
            if variant == 'mirror_time':
                # 합성 시간 재할당: 시뮬레이터 스캔 순서에서 거울 방위 -θ의 취득 시각을 쓴다(위치는 다시 표본화하지 않음).
                shifted = ages+np.degrees(rb)/180.*.1
                margins = .025+follow.measurement_padding_m+follow.object_motion_bound_mps*np.maximum(0., shifted)
            bearings = -bearings
        fresh = last_source is None or scene.source_first_s > last_source
        last_source = scene.source_first_s
        state = c.side_state(points, bearings, status, path, CFG)
        row = {'t': t, 'recorded_reason': detail.get('reason'), 'recorded_speed': output[0],
               'recorded_steering': output[1], 'measured_speed': data['speed_mps']}
        if state['status'] != 'path':
            mode, events = machine.step(state, False, fresh)
            rows.append({**row, 'status': 'no_path', 'mode': mode, 'events': events})
            continue
        p1 = c.p1_path(path, state['ego_offset_m'], state['half_span_m'], CFG)
        age = data['reference_s']-scene.source_first_s
        age_eff = max(age, detail.get('boundary_age_s') or 0.)
        previous_side = machine.side
        mode, events = machine.step(state, p1['valid'], fresh)
        if 'trigger' in events and triggered_at is None:
            triggered_at, trigger_offset = t, state['ego_offset_m']
        toward = state['centering_toward']
        trigger_condition = bool(state['geometry_valid'] and toward and abs(state['ego_offset_m']) >= CFG.min_offset_m
                                 and state['sides'][toward]['points'] >= CFG.min_points)
        row.update(status='path', mode=mode, events=events, trigger_condition=trigger_condition,
                   ego_offset_m=state['ego_offset_m'], yaw_from_path_rad=state['yaw_from_path_rad'],
                   centering_toward=toward, geometry_reasons=state['geometry_reasons'],
                   side_points={k: v['points'] for k, v in state['sides'].items()},
                   side_points_no_ext={k: v['points_without_extension'] for k, v in state['sides'].items()},
                   inlane_rear=state['inlane_rear_points'], unknown=state['side_rear_unknown'],
                   p1_valid=p1['valid'], p1_problems=p1.get('problems'))
        if mode != 'P1':
            p1_prev = None
        if p1['valid'] and (trigger_condition or mode == 'P1'):
            prev = p1_prev if (mode == 'P1' and p1_prev is not None) else data['previous_steering_rad']
            row['p1_check'] = c.existing_checks(p1['path'], points, margins, data['speed_mps'],
                                                prev, age_eff, follow, data['dt_s'])
            row['p1_check']['previous_steering_source'] = 'candidate_chain' if prev is p1_prev else 'recorded_P0'
            if mode == 'P1' and row['p1_check'].get('status') == 'ok':
                p1_prev = row['p1_check']['steering_rad']
        # 기록된 P0 출력과의 일관성: 기록 조향·직전 조향으로 swept를 다시 계산(양수 출력만).
        if output[0] > 0 and detail.get('swept_clearance_m') is not None and variant is None:
            from arena_autonomy.local_path import swept_limit
            again = min(swept_limit(points, a, follow.wheelbase_m, horizon=2.5, margin=margins,
                                    length=follow.length_m, width=follow.width_m)
                        for a in (output[1], data['previous_steering_rad']))
            row['p0_recompute_diff_m'] = again-detail['swept_clearance_m']
        if any(e.startswith('invalid_transition') for e in events):
            alt = c.existing_checks(path, points, margins, data['speed_mps'], data['previous_steering_rad'],
                                    age_eff, follow, data['dt_s'])
            row['invalid_alternatives'] = {
                'P0': {**alt, 'toward_threat': alt.get('status') == 'ok' and c.toward_threat(alt['steering_rad'], previous_side)},
                'HOLD': {'speed_mps': 0.}, 'previous_P1': 'not_valid: no current observation support/frame transform'}
        if variant is None and name == 'e3_onroad_g1' and 4.2 <= t <= 6.6 and triggered_at is not None:
            row['counterfactual'] = counterfactual(path, points, margins, data, age_eff, follow, trigger_offset)
        rows.append(row)
    return rows


def counterfactual(path, points, margins, data, age_eff, follow, l0):
    """정적 반사실: 실제 점을 P1 유지 자세(첫 촉발 시 센서 기준 편차 l0, 경로 정렬)로 옮겨 기존 검사 적용."""
    s, _, _, tangent = c.project(np.zeros((1, 2)), path, 0.)
    frame = c._frame(path, 0.)
    q, seg, length, start, _ = frame
    i = int(np.clip(np.searchsorted(start, s[0], side='right')-1, 0, len(seg)-1))
    foot = q[i]+(s[0]-start[i])/length[i]*seg[i]
    t = seg[i]/length[i]
    normal = np.array([-t[1], t[0]])
    origin = foot+l0*normal
    theta = math.atan2(t[1], t[0])
    rot = np.array([[math.cos(theta), math.sin(theta)], [-math.sin(theta), math.cos(theta)]])
    to_h = lambda xy: (np.asarray(xy)-origin) @ rot.T
    p1 = c.p1_path(path, l0, CFG.width_m/2, CFG)
    if not p1['valid']:
        return {'status': 'p1_invalid', 'problems': p1['problems']}
    check = c.existing_checks(to_h(p1['path']), to_h(points), margins, data['speed_mps'], 0., age_eff, follow, data['dt_s'])
    return {**check, 'kind': 'static_counterfactual', 'offset_m': l0}


def evaluation(base, rows):
    """정답 자세로 측후방 actor 위치를 붙인다(후보 열 계산 뒤, 평가 전용)."""
    from bisect import bisect_right
    ego = [json.loads(l) for l in (ROOT/base/'ego_poses.jsonl').read_text().splitlines()]
    extra = ROOT/base/'additional_actors.json'
    if not extra.exists():
        return
    actor = json.loads(extra.read_text())['poses']['eval_e3_side']
    et, at = [r[0] for r in ego], [r[0] for r in actor]
    for row in rows:
        i, j = bisect_right(et, row['t'])-1, bisect_right(at, row['t'])-1
        if i < 0 or j < 0:
            continue
        _, x, y, yaw = ego[i]
        _, ax, ay, _ = actor[j]
        cy, sy = math.cos(yaw), math.sin(yaw)
        row['eval_actor_rel'] = [cy*(ax-x)+sy*(ay-y), -sy*(ax-x)+cy*(ay-y)]


def summarize(rows):
    path = [r for r in rows if r.get('status') == 'path']
    checks = [r['p1_check'] for r in path if r.get('p1_check', {}).get('status') == 'ok']
    events = {}
    for r in rows:
        for e in r.get('events', []):
            events.setdefault(e, []).append(round(r['t'], 3))
    diffs = [abs(r['p0_recompute_diff_m']) for r in rows if 'p0_recompute_diff_m' in r]
    invalid = [r for r in rows if 'invalid_alternatives' in r]
    problems = {}
    for r in path:
        for p in (r.get('p1_problems') or []):
            problems[p] = problems.get(p, 0)+1
    return {'decisions': len(rows), 'with_path': len(path),
            'trigger_condition': sum(r['trigger_condition'] for r in path),
            'mode_P1': sum(r.get('mode') == 'P1' for r in path),
            'p1_invalid': sum(not r['p1_valid'] for r in path), 'p1_problems': problems,
            'p1_checked': len(checks), 'p1_gap_hold': sum(k['gap_hold'] for k in checks),
            'p1_positive_target': sum(k['target_mps'] > 0 and not k['gap_hold'] for k in checks),
            'p1_clearance_m': (np.percentile([k['swept_clearance_m'] for k in checks], [0, 50, 100]).tolist()
                               if checks else None),
            'events': {k: {'count': len(v), 'first': v[:6]} for k, v in events.items()},
            'invalid_transitions': len(invalid),
            'invalid_P0_toward_threat': sum(r['invalid_alternatives']['P0'].get('toward_threat', False) for r in invalid),
            'p0_recompute_compared': len(diffs), 'p0_recompute_max_abs_diff_m': max(diffs) if diffs else None}


def compact(r):
    keep = ('t', 'recorded_reason', 'recorded_speed', 'mode', 'events', 'ego_offset_m', 'yaw_from_path_rad',
            'side_points', 'unknown', 'p1_valid', 'p1_problems', 'eval_actor_rel')
    out = {k: r.get(k) for k in keep if k in r}
    for k in ('p1_check', 'counterfactual'):
        if k in r:
            out[k] = {x: r[k].get(x) for x in ('status', 'steering_rad', 'swept_clearance_m', 'target_mps', 'gap_hold', 'problems')}
    if 'invalid_alternatives' in r:
        out['invalid_P0'] = {x: r['invalid_alternatives']['P0'].get(x) for x in ('steering_rad', 'toward_threat')}
    return out


def main():
    result = {'scope': 'fixed-version open-loop screening of A2/P1 on stored inputs; candidate columns computed '
                       'with truth files blocked; eval_* only for evaluation; not closed-loop or safety evidence',
              'config': vars(CFG) if hasattr(CFG, '__dict__') else {k: getattr(CFG, k) for k in CFG.__dataclass_fields__},
              'traces': {}, 'counterexamples': {}, 'sources': {}}
    smoke = '--smoke' in sys.argv
    if '--missing-only' in sys.argv:
        # run1 시험기 결함의 원인에 해당하는 대조만 다시 계산한다(저장 3기록 전체 재실행 안 함).
        motion, follow, calls = load_inputs(TRACES['e3_onroad_g1'])
        out = {'scope': 'missing-injection counterexamples only (run1 harness dtype defect fixed)', 'counterexamples': {}}
        for variant in ('missing_all', 'missing_after_trigger'):
            with TruthBlocked():
                vrows = candidate_pass('e3_onroad_g1', motion, follow, calls, G1_WINDOW, variant)
            evaluation(TRACES['e3_onroad_g1'], vrows)
            out['counterexamples'][variant] = {'summary': summarize(vrows),
                                               'timeline': [compact(r) for r in vrows if r.get('status') == 'path']}
        out['sources'] = {f: hashlib.sha256((HERE/f).read_bytes()).hexdigest() for f in ('candidate.py', 'screen.py')}
        (HERE/'report_missing.json').write_text(json.dumps(out, indent=1, ensure_ascii=False, default=float),
                                                encoding='utf-8', newline='\n')
        print(json.dumps({k: v['summary'] for k, v in out['counterexamples'].items()}, indent=1, default=float))
        return
    for name, base in TRACES.items():
        if smoke and name != 'e3_onroad_g1':
            continue
        motion, follow, calls = load_inputs(base)
        with TruthBlocked():
            rows = candidate_pass(name, motion, follow, calls, G1_WINDOW if smoke else None)
        evaluation(base, rows)
        entry = {'summary': summarize(rows)}
        if name.startswith('e3'):
            entry['timeline'] = [compact(r) for r in rows if 3.8 <= r['t'] <= 7.0 or 13.6 <= r['t'] <= 15.2]
            rel = [r for r in rows if 'release_unverified' in r.get('events', [])]
            entry['release_unverified_eval'] = [{'t': r['t'], 'actor_rel': r.get('eval_actor_rel')} for r in rel]
        else:
            entry['trigger_rows'] = [compact(r) for r in rows if r.get('trigger_condition')][:20]
            entry['invalid_rows'] = [compact(r) for r in rows if 'invalid_alternatives' in r][:20]
        if name == 'e3_onroad_g1':
            cf = [r['counterfactual'] for r in rows if 'counterfactual' in r]
            ok = [x for x in cf if x.get('status') == 'ok']
            entry['counterfactual_summary'] = {'rows': len(cf), 'checked': len(ok),
                                               'gap_hold': sum(x['gap_hold'] for x in ok),
                                               'clearance_m': np.percentile([x['swept_clearance_m'] for x in ok],
                                                                            [0, 50, 100]).tolist() if ok else None}
            for variant in ('missing_all', 'missing_after_trigger', 'mirror_static', 'mirror_time'):
                with TruthBlocked():
                    vrows = candidate_pass(name, motion, follow, calls, G1_WINDOW, variant)
                result['counterexamples'][variant] = {'summary': summarize(vrows),
                                                      'timeline': [compact(r) for r in vrows if r.get('status') == 'path'][::5]}
            base_rows = [r for r in rows if G1_WINDOW[0] <= r['t'] <= G1_WINDOW[1] and r.get('status') == 'path']
            mirror = result['counterexamples']['mirror_static']
            result['counterexamples']['mirror_static']['note'] = 'static mirror keeps per-point ages; code symmetry check'
            # 독립성: 같은 창을 정답 차단 상태로 다시 계산해 후보 열이 같은지 확인.
            with TruthBlocked():
                again = candidate_pass(name, motion, follow, calls, G1_WINDOW)
            strip = lambda rs: json.dumps([{k: v for k, v in r.items() if not k.startswith('eval')} for r in rs],
                                          sort_keys=True, default=float)
            window_rows = [r for r in rows if G1_WINDOW[0] <= r['t'] <= G1_WINDOW[1]]
            result['independence'] = {'window_rows': len(window_rows),
                                      'identical_candidate_columns': strip(again) == strip(window_rows)}
        result['traces'][name] = entry
        result['sources'][f'{base}/follow_decisions.jsonl'] = hashlib.sha256(
            (ROOT/base/'follow_decisions.jsonl').read_bytes()).hexdigest()
    for f in ['src/arena_autonomy/arena_autonomy/local_follow.py', 'src/arena_autonomy/arena_autonomy/local_path.py',
              'src/arena_autonomy/arena_autonomy/occupancy_timing.py', 'src/arena_autonomy/arena_autonomy/scene_motion.py',
              'src/arena_autonomy/arena_autonomy/local_scene.py', 'src/arena_autonomy/arena_autonomy/lidar_motion.py',
              'src/arena_autonomy/arena_autonomy/follow_trace.py']:
        result['sources'][f] = hashlib.sha256((ROOT/f).read_bytes()).hexdigest()
    for f in ('candidate.py', 'screen.py'):
        result['sources'][f] = hashlib.sha256((HERE/f).read_bytes()).hexdigest()
    (HERE/('smoke_report.json' if smoke else 'report.json')).write_text(json.dumps(result, indent=1, ensure_ascii=False, default=float),
                                    encoding='utf-8', newline='\n')
    brief = {k: v['summary'] for k, v in result['traces'].items()}
    brief['counterexamples'] = {k: v['summary'] for k, v in result['counterexamples'].items()}
    brief['independence'] = result.get('independence')
    brief['g1_counterfactual'] = result['traces'].get('e3_onroad_g1', {}).get('counterfactual_summary')
    print(json.dumps(brief, indent=1, ensure_ascii=False, default=float))


if __name__ == '__main__':
    main()

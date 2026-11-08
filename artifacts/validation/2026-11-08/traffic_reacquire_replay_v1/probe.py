"""ON(standing_wall_reacquire=True) 재생 중 local_follow.local_path 호출과 _baseline_command 핵심 분기를 일시 계측한다.

0~34초 prefix 전체를 재생해 상태를 유지하고, 27.8~29.3초와 33~34초의 새 스캔에서만 기록한다.
제어 조건/임계값은 바꾸지 않는다. 계측은 이 프로세스 안의 함수 래핑과 sys.settrace뿐이다.
세그먼트 분해는 local_path 연결/앵커 규칙의 진단용 복제이며, local_path 사유와의 일치를 매번 검사한다.
상대 차량 위치는 평가용 정답(actor_poses)이며 제어기에 입력하지 않는다.
"""
import dataclasses, hashlib, json, math, sys, inspect
from pathlib import Path
import numpy as np
import arena_autonomy.local_follow as lf
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig

HERE = Path(__file__).resolve().parent
RUN = HERE.parent/'traffic_lap_newcourse_v1'/'two_actor_course_lap'/'run1'
SOURCE = RUN/'follow_decisions.jsonl'
WINDOWS = ((27.8, 29.3), (33.0, 34.0))
END_S = 34.0

src_lines, first = inspect.getsourcelines(lf.LocalFollow._baseline_command)


def line_of(text, after=None):
    hits = [first+i for i, l in enumerate(src_lines) if text in l and (after is None or first+i > after)]
    assert hits, text
    return hits[0]


L_ELIG = line_of("eligible = sufficient(candidate, info)")
L_GATE = line_of("if eligible and carried is None and not bilateral:")
L_STAND = line_of("if not eligible and cfg.standing_wall_reacquire")
L_STAND_SET = line_of("detail['bilateral_reference'] = 'standing_preferred_wall'")
L_FINAL = line_of("detail['fresh_boundary_accepted'] =")
L_HOLD = line_of("return self._hold('follow_boundary_unconfirmed', detail)")
WATCH = {L_ELIG: 'initial_eligibility', L_GATE: 'bilateral_gate', L_STAND: 'standing_option_check',
         L_STAND_SET: 'standing_option_applied', L_FINAL: 'final_eligibility', L_HOLD: 'hold_unconfirmed'}
CODE = lf.LocalFollow._baseline_command.__code__


def summary_info(info):
    return None if info is None else {k: info.get(k) for k in ('reason', 'wall_model', 'boundary_samples',
                                                            'boundary_observed_ahead_m', 'wall_distance_m')}


def mirror_segments(points, side, offset, horizon):
    """local_path의 연결 분할·앵커 규칙 복제(rays=None 경로). 진단 전용."""
    sign = 1 if side == 'left' else -1
    p = np.asarray(points, float)
    p = p[np.all(np.isfinite(p), axis=1)]
    if len(p) < 12:
        return []
    p = p[np.argsort(-sign*np.arctan2(p[:, 1], p[:, 0]))]
    gaps = np.linalg.norm(np.diff(p, axis=0), axis=1)
    limits = .10+.035*np.minimum(np.linalg.norm(p[:-1], axis=1), np.linalg.norm(p[1:], axis=1))
    out = []
    for wall in np.split(p, np.flatnonzero(gaps > limits)+1):
        seg = {'n': len(wall), 'x_min': float(wall[:, 0].min()), 'x_max': float(wall[:, 0].max()),
               'y_min': float(wall[:, 1].min()), 'y_max': float(wall[:, 1].max()),
               'start_xy': wall[0].round(3).tolist(), 'end_xy': wall[-1].round(3).tolist()}
        cand = np.flatnonzero((np.abs(wall[:, 0]) < .35) & (sign*wall[:, 1] > .10))
        seg['anchor_zone_points'] = int(len(cand))
        if len(wall) >= 12 and len(cand):
            d = np.linalg.norm(wall[cand]-[0., sign*offset], axis=1)
            a = int(cand[np.argmin(d)])
            seg['anchor_score_m'] = float(d.min())
            seg['anchor_xy'] = wall[a].round(3).tolist()
            chord, middle = wall[2:]-wall[:-2], wall[1:-1]-wall[:-2]
            rough = float(np.median(np.abs(chord[:, 0]*middle[:, 1]-chord[:, 1]*middle[:, 0]) /
                                    np.maximum(1e-9, np.linalg.norm(chord, axis=1))))
            metric = wall
            if rough > .003:
                padded = np.pad(wall, ((3, 3), (0, 0)), mode='edge')
                metric = np.column_stack([np.convolve(padded[:, k], np.ones(7)/7, mode='valid') for k in (0, 1)])
            arc = np.r_[0., np.cumsum(np.maximum(1e-6, np.linalg.norm(np.diff(metric, axis=0), axis=1)))]
            seg['ahead_m'] = float(min(arc[-1], arc[a]+horizon)-arc[a])
            seg['eligible_mirror'] = bool(seg['anchor_score_m'] <= .35 and seg['ahead_m'] >= .28)
        else:
            seg['eligible_mirror'] = False
            seg['why_not'] = 'fewer_than_12_points' if len(wall) < 12 else 'no_point_in_anchor_zone'
        out.append(seg)
    return out


state = {'active': False, 'calls': [], 'events': [], 'npoints': None}
orig_local_path = lf.local_path


def traced_local_path(points, side, offset=.425, horizon=1.8, **kw):
    path, info = orig_local_path(points, side, offset, horizon, **kw)
    if state['active']:
        segs = mirror_segments(points, side, offset, horizon)
        state['calls'].append({'order': len(state['calls']), 'side': side, 'cloud_points': int(len(points)),
                               'full_cloud': state['npoints'] == len(points), 'horizon_m': horizon,
                               'path_points': None if path is None else int(len(path)), 'info': summary_info(info),
                               'segments_total': len(segs),
                               'segments_mirror_eligible': sum(s['eligible_mirror'] for s in segs),
                               'mirror_consistent': (info.get('reason') == 'wall_segment_short') ==
                                                    (not any(s['eligible_mirror'] for s in segs)),
                               'segments_with_anchor_or_12plus': [s for s in segs if s['n'] >= 12 or s['anchor_zone_points']]})
    return path, info


lf.local_path = traced_local_path


def tracer(frame, event, arg):
    if frame.f_code is not CODE:
        return None

    def local(frame, event, arg):
        if event == 'line' and frame.f_lineno in WATCH and state['active']:
            v = frame.f_locals
            info = v.get('info')
            state['npoints'] = len(v['points']) if 'points' in v else state['npoints']
            state['events'].append({'line': frame.f_lineno, 'label': WATCH[frame.f_lineno],
                                    'eligible': v.get('eligible'), 'carried_is_none': v.get('carried') is None,
                                    'bilateral': v.get('bilateral'), 'corroborated': v.get('corroborated'),
                                    'speed': v.get('speed'), 'candidate_wall': v.get('candidate_wall'),
                                    'side': v.get('side'), 'info': summary_info(info) if isinstance(info, dict) else None,
                                    'preferred_info': summary_info(v.get('preferred_info')) if isinstance(v.get('preferred_info'), dict) else None,
                                    'pending_count': getattr(v.get('self'), 'pending_count', None)})
        return local
    if state['active']:
        # observed_path 첫 호출 전에 전체 점 수를 알기 위해 진입 시점의 scene에서 계산
        sc = frame.f_locals.get('scene')
        if sc is not None:
            state['npoints'] = len(np.asarray(sc.occupied_endpoints_xy_m, float).reshape(-1, 2))
    return local


def pose_at(series, t):
    i = min(range(len(series)), key=lambda k: abs(series[k][0]-t)) if len(series) < 50 else None
    if i is None:
        lo, hi = 0, len(series)-1
        while lo < hi:
            mid = (lo+hi)//2
            if series[mid][0] < t:
                lo = mid+1
            else:
                hi = mid
        i = lo
    return series[i]


def relative(ego, other):
    dx, dy = other[1]-ego[1], other[2]-ego[2]
    c, s = math.cos(ego[3]), math.sin(ego[3])
    return {'x_m': dx*c+dy*s, 'y_m': -dx*s+dy*c, 'yaw_rel_rad': other[3]-ego[3], 'pose_t': other[0]}


def main():
    ego = [json.loads(l) for l in open(RUN/'ego_poses.jsonl')]
    lead = [json.loads(l) for l in open(RUN/'actor_poses.jsonl')]
    second = json.load(open(RUN/'additional_actors.json'))['poses']['eval_second_lead']
    sha_before = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    ctl = mc = None
    samples, mismatches, calls = [], 0, 0
    with open(SOURCE, encoding='utf-8') as f:
        for line in f:
            e = json.loads(line)
            if e['kind'] == 'header':
                h = e['data']
                mc = MotionConfig(**h['motion_config'])
                cfg = dataclasses.replace(FollowConfig(**h['follow_config']), standing_wall_reacquire=True)
                ctl = BypassHandoffFollow(cfg, HandoffExecutionConfig.from_mapping(h['execution_config']),
                                          allow_simulation_trial=True)
                continue
            if e['kind'] == 'reset':
                ctl.reset()
                continue
            if e['kind'] != 'call':
                break
            d = e['data']
            t = d['input']['reference_s']
            if t > END_S:
                break
            inside = any(a <= t <= b for a, b in WINDOWS)
            state.update(active=inside, calls=[], events=[], npoints=None)
            if inside:
                sys.settrace(tracer)
            try:
                out = freeze(replay_call(ctl, d['input'], mc))
            finally:
                sys.settrace(None)
                state['active'] = False
            calls += 1
            same = equal_output(comparable_decision(out), comparable_decision(d['output']))
            mismatches += not same
            if inside and state['calls']:
                n = out[2].get('normal', {}) if isinstance(out[2], dict) else {}
                cc = n.get('corridor_candidate') or {}
                ego_p = pose_at(ego, t)
                samples.append({
                    'seq': e['seq'], 't': t, 'measured_speed_mps': d['input']['speed_mps'],
                    'scan_source_first_s': n.get('source_first_s'), 'output_equals_recorded': same,
                    'speed': out[0], 'steer': out[1], 'reason': n.get('reason'),
                    'boundary_candidate_reason': n.get('boundary_candidate_reason'),
                    'boundary_selection': n.get('boundary_selection'), 'bilateral_reference': n.get('bilateral_reference'),
                    'fragmented_reason': (n.get('fragmented_boundary_evidence') or {}).get('reason'),
                    'corner_distance_evidence': {k: v for k, v in (n.get('corner_distance_evidence') or {}).items()
                                                 if not isinstance(v, (list, dict))},
                    'corridor_candidate': {k: cc.get(k) for k in ('positive', 'reason', 'role_selection', 'preferred_side',
                                                                  'preferred_observed_path_agrees') if k in cc},
                    'local_path_calls': state['calls'], 'branch_events': state['events'],
                    'eval_only_truth': {'ego': ego_p, 'lead_rel': relative(ego_p, pose_at(lead, t)),
                                        'second_rel': relative(ego_p, pose_at(second, t))}})
    result = {'source_sha256_before': sha_before,
              'source_sha256_after': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              'calls_replayed': calls, 'on_vs_recorded_mismatches': mismatches,
              'windows_s': WINDOWS, 'watched_lines': {str(k): v for k, v in WATCH.items()},
              'mirror_all_consistent': all(c['mirror_consistent'] for s in samples for c in s['local_path_calls']),
              'fresh_samples': len(samples), 'samples': samples,
              'scope': 'stored inputs; ON option only; instrumentation in-process; truth is evaluation-only'}
    (HERE/'probe_result.json').write_text(json.dumps(result, indent=1, ensure_ascii=False, default=lambda o: o.item() if hasattr(o, 'item') else str(o))+'\n', encoding='utf-8')
    print(json.dumps({k: result[k] for k in ('calls_replayed', 'on_vs_recorded_mismatches', 'mirror_all_consistent',
                                              'fresh_samples', 'watched_lines')}, ensure_ascii=False))
    return 0 if result['source_sha256_before'] == result['source_sha256_after'] else 1


if __name__ == '__main__':
    sys.exit(main())

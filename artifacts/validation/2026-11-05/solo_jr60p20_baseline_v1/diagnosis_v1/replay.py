"""run1 저장 입력 재생: 42–44 m 속도 제한 체인 분해와 후보 A(회랑 다리→기본 경로 재개 연속) 비교.

평가·재생 전용. 격리 사본(isolate.sh, 등록 해시 일치)을 PYTHONPATH로 쓴다. 런타임 소스를 바꾸지 않으며
후보는 이 프로세스 메모리 안의 하위 클래스다. 저장 궤적은 고정이므로 명령 차이를 랩 시간 단축으로 부르지 않는다.
사용(WSL): run_replay.sh (격리 사본 PYTHONPATH로 python3 -B replay.py)
"""
import json
import math
from pathlib import Path
import sys

import numpy as np

import arena_autonomy.local_follow as lf
from arena_autonomy.follow_trace import replay_call, replay_events, freeze, equal_output, comparable_decision
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow

HERE = Path(__file__).resolve().parent
RUN = HERE.parent/'run1'
WINDOW = (37.0, 40.2)
ACCEL, DT, LAT = 1.5, .02, 3.0   # 부모 노드: local_fast 계열 acceleration_mps2, 50 Hz, lateral_acceleration_limit_mps2


def events():
    with (RUN/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


class BridgeRelease(LocalFollow):
    """후보 A. 회랑 다리가 직전 호출까지 양수로 주행했고, 기본 경로가 같은 오른쪽/선호 벽 경로를 이미 다시 받아
    재개 대기(follow_release_wait)만 남았을 때, 다리가 꺼진 그 호출에 0 대신 기본 경로 자체 목표를 낸다.

    조건: hold 사유가 follow_boundary_unconfirmed(경로 근거 전환), 재개 확인 1회 이상, 쓸림 여유 >= resume_clearance,
    기본 목표 > 0. 기본 경로의 곡률·제동·점유·부분 관측 상한은 그대로 뒤에서 적용된다(_partial_command).
    관측 정지·점유 정지·결측 정지의 재개 대기는 바꾸지 않는다.
    """

    def reset(self):
        super().reset()
        self.bridged_last = False

    def _path_command(self, scene, history, speed, side, previous_steering=0., control_dt=.02, *, allow_partial=False):
        out = super()._path_command(scene, history, speed, side, previous_steering, control_dt,
                                    **({'allow_partial': True} if allow_partial else {}))
        detail = out[2]
        bridged = getattr(self, 'bridged_last', False)
        if (out[0] == 0. and bridged and detail.get('reason') == 'follow_release_wait'
                and self.hold_reason == 'follow_boundary_unconfirmed' and self.release_count >= 1
                and (detail.get('swept_clearance_m') or 0.) >= self.config.resume_clearance_m
                and (detail.get('target_speed_before_hold_mps') or 0.) > 0.):
            out = (float(detail['target_speed_before_hold_mps']), out[1],
                   {**detail, 'reason': 'local_follow_bridge_release_candidate', 'behavior': 'FOLLOW',
                    'underlying_reason': 'follow_release_wait'})
        self.bridged_last = bool(out[0] > 0. and (out[2].get('corridor_selected') or
                                 out[2].get('reason') == 'local_follow_bridge_release_candidate'))
        return out


class Probe:
    """local_follow 모듈 전역 함수 호출값을 호출 단위로 모은다(메모리 계측, 출력 불변)."""

    def __init__(self):
        self.calls = []
        self.originals = {name: getattr(lf, name) for name in ('pursue_path', 'braking_speed')}

        def pursue(*a, **k):
            result = self.originals['pursue_path'](*a, **k)
            self.calls.append(('pursue', dict(cap=result[0], remaining=result[2], speed_cap=k.get('speed_cap'))))
            return result

        def braking(clearance, age, *rest):
            result = self.originals['braking_speed'](clearance, age, *rest)
            self.calls.append(('braking', dict(distance=clearance, age=age, speed=result)))
            return result
        lf.pursue_path, lf.braking_speed = pursue, braking

    def take(self):
        calls, self.calls = self.calls, []
        return calls


def run(follow_type, probe=None, keep=None):
    count, header, follow, motion = 0, None, None, None
    rows, mismatches = [], 0
    for event in events():
        kind, data = event['kind'], event['data']
        if kind == 'header':
            header = data
            motion = MotionConfig(**data['motion_config'])
            follow = follow_type(FollowConfig(**data['follow_config']))
        elif kind == 'reset':
            follow.reset()
        elif kind == 'call':
            if probe:
                probe.take()
            actual = freeze(replay_call(follow, data['input'], motion))
            same = equal_output(comparable_decision(actual), comparable_decision(data['output']))
            mismatches += not same
            t = data['input']['reference_s']
            row = dict(t=t, out=actual[0], steer=actual[1], reason=actual[2].get('reason'), recorded=data['output'][0],
                       recorded_reason=data['output'][2].get('reason'), same=same)
            if keep and keep[0] <= t <= keep[1]:
                d = actual[2]
                po = d.get('partial_observation') or {}
                row.update(behavior=d.get('behavior'), underlying=d.get('underlying_reason'), side=data['input']['side'],
                           path_wall=d.get('path_wall'), open_route=d.get('open_route_speed_enabled'),
                           boundary_selection=d.get('boundary_selection'), fresh_accepted=d.get('fresh_boundary_accepted'),
                           corridor_positive=(d.get('corridor_candidate') or {}).get('positive'),
                           corridor_reason=(d.get('corridor_candidate') or {}).get('reason'),
                           clearance=d.get('swept_clearance_m'), before_hold=d.get('target_speed_before_hold_mps'),
                           turn_preview=d.get('turn_preview'), constant_arc=d.get('constant_arc_target_mps'),
                           partial_reason=po.get('reason'), partial_cap=po.get('speed_cap_mps'),
                           unknown=po.get('unknown_samples'), max_gap_deg=po.get('max_gap_deg'),
                           gap_neighbor_m=po.get('minimum_gap_neighbor_m'), measured=data['input']['speed_mps'],
                           age=d.get('boundary_age_s'))
                if probe:
                    calls = probe.take()
                    pursue = [c for n, c in calls if n == 'pursue']
                    brake = [c for n, c in calls if n == 'braking']
                    if pursue:
                        row['path_cap'] = pursue[-1]['cap']
                        row['path_cap_open_route'] = pursue[-1]['speed_cap']
                    if brake:
                        row['occupancy_braking'] = brake[-1]['speed']
                    if row.get('clearance') is not None:
                        row['headway'] = max(0., row['clearance']-follow.config.standstill_gap_m)/follow.config.headway_s
            rows.append(row)
            count += 1
    return dict(calls=count, mismatches=mismatches, rows=rows, config=header['follow_config'])


def binding(row):
    if row['out'] <= .001:
        return 'zero:'+str(row['reason'])
    if row['reason'] == 'local_follow_corridor_candidate':
        return 'corridor_bridge_proposal'
    if row.get('partial_cap') is not None and abs(row['partial_cap']-row['out']) < 2e-3:
        return 'partial_cap'
    # 회전 미리보기는 일정 원호 목표(점유 제동·차간)를 바꾼 값이다. 출력이 그 값이면 미리보기가 정한 것이다.
    if (row.get('turn_preview') and row.get('before_hold') is not None and row.get('constant_arc') is not None
            and abs(row['before_hold']-row['out']) < 2e-3 and abs(row['before_hold']-row['constant_arc']) > 1e-6):
        return 'turn_preview:'+row['turn_preview']
    terms = {k: row.get(k) for k in ('path_cap', 'occupancy_braking', 'headway') if row.get(k) is not None}
    if not terms:
        return 'unknown'
    name = min(terms, key=terms.get)
    return name if abs(terms[name]-row['out']) < 2e-3 else 'other'


def parent(rows, lat=True):
    """부모 노드 출력 모델: min(계획, 직전+1.5 m/s²·dt, sqrt(3/곡률)). 실제 /drive와의 대조는 README."""
    last, out = 0., []
    for r in rows:
        v = r['out']
        curvature = abs(math.tan(r['steer'])/.135)
        if lat and curvature > 1e-6:
            v = min(v, math.sqrt(LAT/curvature))
        v = min(v, last+ACCEL*DT)
        out.append(v)
        last = v
    return out


def main():
    baseline_check = replay_events(events())
    probe = Probe()
    now = run(LocalFollow, probe, WINDOW)
    for name, func in probe.originals.items():
        setattr(lf, name, func)
    cand = run(BridgeRelease)
    rows = now['rows']
    window = [r for r in rows if WINDOW[0] <= r['t'] <= WINDOW[1]]
    traj = [json.loads(x) for x in (RUN/'trajectory.jsonl').read_text(encoding='utf-8').splitlines() if x.strip()]
    tt, ss = np.array([x['time'] for x in traj]), np.array([x['progress_m'] for x in traj])
    bands = {}
    for r in window:
        r['binding'] = binding(r)
        r['progress_m'] = float(np.interp(r['t'], tt, ss))
        edge = next((e for e in (42., 43., 43.4, 44., 99.) if r['progress_m'] < e))
        key = f"<{edge:g}m" if edge < 99 else '>=44m'
        bands.setdefault(key, {}).setdefault(r['binding'], 0.)
        bands[key][r['binding']] += DT
    changed = [dict(t=round(a['t'], 3), recorded=a['out'], candidate=b['out'], recorded_reason=a['reason'], candidate_reason=b['reason'],
                    steer_delta=b['steer']-a['steer'])
               for a, b in zip(rows, cand['rows']) if not equal_output(a['out'], b['out']) or not equal_output(a['steer'], b['steer'])]
    pub_now, pub_cand = parent(rows), parent(cand['rows'])
    times = [r['t'] for r in rows]
    lo = next(i for i, t in enumerate(times) if t >= WINDOW[0])
    hi = max(i for i, t in enumerate(times) if t <= WINDOW[1])
    deficit = sum((b-a)*DT for a, b in zip(pub_now[lo:hi+1], pub_cand[lo:hi+1]))
    # 분해 전용(제안 아님): 같은 모델에서 38.70–38.80 s의 결측 정지·재개 대기를 직전 0.5 m/s 상한으로 바꿨다면.
    apportion = [dict(r) for r in cand['rows']]
    for r in apportion:
        if 38.69 <= r['t'] <= 38.81 and r['out'] <= .001:
            r['out'] = .5
    pub_app = parent(apportion)
    deficit_gap = sum((b-a)*DT for a, b in zip(pub_cand[lo:hi+1], pub_app[lo:hi+1]))
    zeros = [dict(t=round(r['t'], 3), reason=r['reason']) for r in rows if r['out'] <= .001 and r['t'] > 9.]
    cand_zeros = [dict(t=round(r['t'], 3), reason=r['reason']) for r in cand['rows'] if r['out'] <= .001 and r['t'] > 9.]
    triggers = [c for c in changed if c['candidate_reason'] == 'local_follow_bridge_release_candidate']
    result = dict(
        scope='저장 입력 재생(평가 전용). 격리 사본·메모리 후보. 궤적 고정이라 명령 차이는 폐루프 랩 단축이 아니다.',
        baseline_replay=dict(calls=baseline_check['calls'], matched=baseline_check['matched'],
                             passed=baseline_check['passed']),
        instrumented_replay_mismatches=now['mismatches'],
        zero_calls_after_start=zeros,
        binding_time_by_progress_s={k: {n: round(x, 2) for n, x in v.items()} for k, v in bands.items()},
        window_rows=[{k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()} | dict(published_model=round(p, 4))
                     for r, p in zip(rows[lo:hi+1], pub_now[lo:hi+1])],
        candidate_A=dict(
            definition=BridgeRelease.__doc__.strip(),
            calls=cand['calls'], changed_calls=len(changed), triggers=len(triggers), changed=changed[:40],
            zero_calls_after_start=cand_zeros,
            published_model_distance_gain_in_window_m=round(deficit, 4),
            note='부모 증속 모델에서 창 안 명령 적분 차이. 실제 차체 지연·이후 입력 변화 미반영(개루프).'),
        apportion_only_gap_hold=dict(
            published_model_distance_gain_in_window_m=round(deficit_gap, 4),
            note='분해 전용. 38.70–38.80 s 결측 정지를 0.5 m/s로 둔 가정. 제안이 아니며 판정선 변경 근거가 아니다.'),
        parent_model=dict(acceleration_mps2=ACCEL, dt_s=DT, lateral_acceleration_mps2=LAT, wheelbase_m=.135))
    (HERE/'replay.json').write_text(json.dumps(result, ensure_ascii=False, indent=1, default=str), encoding='utf-8', newline='\n')
    print(json.dumps(dict(baseline=result['baseline_replay'], instrumented_mismatches=now['mismatches'],
                          zeros=zeros, cand_zeros=cand_zeros, changed=len(changed), triggers=len(triggers),
                          gain_m=round(deficit, 4), gap_gain_m=round(deficit_gap, 4)), ensure_ascii=False))
    return 0 if baseline_check['passed'] and now['mismatches'] == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())

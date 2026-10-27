"""옆차 통과 뒤 기존 G1 출력을 허용할지 고르는 상태 선택 시제품. 기본 OFF이며 기록된 G1 시뮬레이션에서만 명시적으로 연결한다.

입력은 현재 장면(운동 보정 스캔), 기존 G1 출력, 같은 입력의 평행 경로 후보 출력, 측정 속도·직전 조향과 이 선택기가
지난 판단에서 남긴 센서 기억뿐이다. 지도·actor 정답·그리드 번호·고정 전환 시각은 쓰지 않는다. 경로·점유 팽창·독립 보호는
기존 그대로이며, 이 선택기는 둘 중 하나의 기존 출력을 고를 뿐 새 경로를 만들지 않는다.

상태
- SOLO: 측방 위협을 한 번도 보지 않았다. 기존 G1 출력을 그대로 쓴다(통과 증거를 기다리는 역설 없음).
- GUARD: 어느 쪽이든 위협을 보았고 그 쪽의 통과 증거가 아직 없다. 평행 후보 출력(현재 위치 유지 또는 HOLD)을 쓴다.
- RELEASED: 본 쪽 모두 통과 증거가 있다. 기존 G1 출력의 실제 예측 차체 영역을 검사해 허용하거나 평행 후보로 돌아간다.

통과 증거(측면 σ, 물체 동일성 가정 없음)
1. 위협 판정은 M3 `side_threats`(벽 오프셋 ±3 cm 제외·압축 반환 3점 이상)를 그대로 쓴다.
2. 위협이 사라질 때 마지막 위협 장면의 최후단 반환 ds가 자차 앞면+`pass_front_margin_m` 이상이면 `benign`, 아니면
   `rear`(뒤·옆에서 사라짐: 후방 가림·결측과 구별 불가)로 남긴다. 한 번 `rear`가 되면 그 쪽은 해당 세션 동안 `benign`으로
   바뀌지 않는다(다른 물체가 앞으로 지나간 것으로 지울 수 없음).
3. 그 뒤 서로 다른 fresh 스캔 `release_fresh`회 연속, σ의 옆면 밖 띠(후면~앞면, 옆면+1 cm~+15 cm, 벽 안쪽)가 마스크 외
   모든 표본에서 양의 자유 관측이어야 한다. 양의 관측이 0개인 띠는 승인하지 않는다. 차체 가림 마스크 표본은 자유로
   바꾸지 않고 따로 센다. `benign`인 쪽은 이 띠로 통과를 승인한다. `rear`/`unknown`은 빈 띠만으로 해제하지 않는다.
4. `rear`/`unknown` 기록은 지우지 않는다. 같은 띠를 측방 위협 창 전체(자차 원점 기준 뒤 `band_behind_m`~앞 `band_ahead_m`)
   로 늘린 영역이 같은 규칙(마스크 외 전부 양의 관측, 양의 관측 1개 이상)으로 fresh `release_fresh`회 연속 확인될 때만
   통과로 본다. 즉 사라진 물체가 측방 창의 관측 가능한 부분 어디에도 없다는 현재 관측이다. 벽과 한 묶음이 되어 compact
   판정이 꺼졌지만 아직 옆에 있는 물체는 그 뒤 표본을 가리므로 승인되지 않는다. 마스크(자차 뒤 원뿔)에 숨은 물체의
   부재는 증명하지 않으며, 해제 뒤 기존 G1 출력은 아래의 같은 차체 이동 검사를 받는다(ADR 0027 작은 후방 이동 재검토).

RELEASED의 기존 G1 출력 허용
- 속도 0이면 그대로(HOLD).
- 현재/직전 조향의 반응+제동 구간 차체 모서리 횡변위가 `small_excursion_m` 이하이면 허용한다. 이는 기존 평행 후보와 같은
  한계이며, 후방 마스크의 뒤차·꼬리 띠 미세 물체 위험은 해소하지 않은 채 남긴다(ADR 0027의 작은 이동 재검토 범위).
- 더 크면 두 조향의 정곡률 원호 차체·바퀴 외곽 중 현재 차체 밖 표본을 현재 스캔으로 검사한다. 차체 모서리의 x/y
  극점 자세를 추가하고 일반 미관측 표본은 거부한다. 차체 가림 마스크의 움직임은 현재 차체 둘레 1 cm 안에만
  남는 경우를 별도 제한 위험으로 허용한다. 그보다 큰 가림 이동은 거부한다. 통과 기록을 현재 자유 관측으로 바꾸지 않는다.
  거부되면 평행 후보 출력을 쓴다. 표본 자세 사이 연속 영역과 숨은 물체의 미래 접근은 여전히 미입증이다.

GUARD 곡선 후보(`guard_curve_trial`, 기본 OFF·통과 선택기 필요)
- GUARD에서 평행 후보가 알려진 기하 실패(`reference_not_straight`)로 정지했을 때만 기존 G1 출력을 별도 저속 후보로
  검사한다. 평행 후보가 진행 가능하면 그대로 쓴다. 기존 G1이 정상 양수일 때만이며 전방 간격 정지·입력 거절·STOP은 따른다.
- 목표는 min(G1, 0.35 m/s). 측정 속도가 더 크면 그 속도로 예측한다. 해제되지 않은 쪽마다 차체 모서리의 경로 횡위치가
  GUARD 진입 때보다 1 cm 넘게 그 쪽으로 더 접근하는 예측은 거절한다(누적 접근 포함). 이미 기준 밖이더라도 현재보다
  접근을 늘리지 않는 진행/회복은 후보로 남긴다. 현재/이전 조향의 이동 영역은 작은 이동에서도 검사한다.
  상태는 GUARD로 남고 통과 증거·vanish 기록은 바꾸지 않는다. 그 방향 영역을 검사했다는 뜻이지 옆 물체나
  숨은 물체의 위험을 지웠다는 뜻이 아니다.
- 후반부 옆면 1 mm 이내 표본이 좁은 무반환에만 걸리면 별도 위험 후보로 분류한다. 원시 +inf와 양옆 유효 반환,
  3도 이하 공백·0.6 m 이상 이웃·가까운 점유 부재를 함께 확인한다. unobserved 개수와 후방 위험은 그대로 기록하며
  자유 공간으로 채우지 않는다. 큰 이동·전방·마스크 경계·유효한 가까운 반환을 이 예외로 보내지 않는다.

고장: 시각 역행·세대 변경·장면 나이 0.3 s 이상·기존 G1 입력 정지면 이번 판단은 HOLD(양수 금지)이다. 위협을 본 적이 있으면
양쪽을 `unknown`으로 표시하고 새 통과 증거를 다시 요구한다. 해제 뒤 같은 쪽 위협이 다시 보이면 GUARD로 돌아간다.
유한 음수 속도(제동 중 역회전)는 정지를 유지한다. SOLO에서 기존 G1도 이미 정지를 냈다면 그 출력(조향 포함)을 그대로 쓴다.
"""
from dataclasses import dataclass
import math

import numpy as np

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.lateral_intent import (LateralIntentConfig, ObservationGrid, cell_coordinates, side_threats)
from arena_autonomy.m3_observed_follow import command_arc
from arena_autonomy.observation_support import depth_support
from arena_autonomy.rear_fringe_trial import bounded_rear_fringe
from arena_autonomy.parallel_lane_trial import lateral_excursion

NORMAL_BASELINE = ('local_follow', 'follow_gap_hold', 'follow_release_wait', 'local_follow_corridor_candidate')
# GUARD 곡선 후보를 검사하는 알려진 평행 후보 기하 실패와 양수 기존 G1 출력 이유(그 밖의 정지·입력 거절은 그대로 따른다).
GUARD_GEOMETRY_REASONS = ('parallel_hold_reference_not_straight',)
GUARD_BASELINE_REASONS = ('local_follow', 'local_follow_corridor_candidate')
SIDES = (('left', 1.), ('right', -1.))


@dataclass(frozen=True)
class SelectorConfig:
    release_fresh: int = 2            # 기존 두 fresh 확인 관례(새 근거값 아님)
    small_excursion_m: float = .01    # 기존 평행 후보의 차체 모서리 횡변위 한계
    pass_front_margin_m: float = .10  # 최후단 반환이 자차 앞면보다 앞선 거리(시험값, C1 거리 오차 3 cm 포함 여유)
    band_width_m: float = .15         # 옆면 밖 확인 띠 폭(규정 최대 차폭)
    band_offset_m: float = .01        # 옆면에서 띠 시작(외곽 표본과 차체 경계 중복 회피)
    band_step_m: float = .02
    max_scene_age_s: float = .3       # 기존 평행 후보·M3와 같은 장면 나이 상한
    masked_tail_allowance_m: float = .01  # 현재 차체 둘레의 작은 가림 이동. 자유 관측으로 표시하지 않음
    guard_speed_cap_mps: float = .35      # GUARD 곡선 후보 상한. 기존 평행 후보와 같은 시험 상한(새 근거값 아님)

    def __post_init__(self):
        if type(self.release_fresh) is not int or self.release_fresh < 2:
            raise ValueError('release requires at least two distinct scans')
        for key, value in vars(self).items():
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('selector constants must be finite and positive')
        if (self.band_offset_m >= self.band_width_m or self.max_scene_age_s > .3
                or self.masked_tail_allowance_m > self.small_excursion_m or self.guard_speed_cap_mps > .35):
            raise ValueError('invalid band or scene age')


def lateral_config(follow_config):
    c = follow_config
    return LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass', wheelbase_m=c.wheelbase_m,
                               length_m=c.length_m, width_m=c.width_m, max_steering_rad=c.max_steering_rad)


def threat_view(scene, path_points, cfg):
    """M3와 같은 측방 위협 판정과 측면별 반환 ds. 경로가 없으면 None."""
    path = np.asarray(path_points if path_points is not None else [], dtype=float).reshape(-1, 2)
    if len(path) < 2 or not np.all(np.isfinite(path)):
        return None
    try:
        frame = mx.ExecutionFrame(path)
    except (ValueError, TypeError):
        return None
    state = mx.ego_state(frame, cfg)
    light = mx.ReturnGrid(scene, cfg)
    ds, masks = mx.side_return_masks(frame, state, light, cfg)
    if any(np.any(m) for m in masks):
        grid = ObservationGrid(scene, cfg)
        s, l = cell_coordinates(grid, frame)
        observed = grid.observed_free()
    else:
        grid, s, l, observed = light, None, None, None
    threats = side_threats(frame, state, grid, cfg, s, l, observed)
    out = {'frame': frame, 'state': state}
    for (name, _), mask in zip(SIDES, masks):
        values = ds[mask] if len(ds) else np.zeros(0)
        out[name] = {'present': bool(threats[name]['present']), 'points': int(threats[name]['points']),
                     'rearmost_ds_m': float(np.min(values)) if len(values) else None,
                     'frontmost_ds_m': float(np.max(values)) if len(values) else None}
    return out


def band_check(scene, frame, sign, cfg, sel, longitudinal=None):
    """옆면 밖 띠의 양의 자유 관측. 마스크 표본은 자유로 세지 않고 따로 센다.

    longitudinal(뒤, 앞)을 주면 차체 길이 대신 그 자차 좌표 구간으로 띠를 늘린다(rear/unknown 해제용 측방 창).
    """
    lo, hi = longitudinal if longitudinal is not None else (-cfg.length_m/2, cfg.length_m/2)
    xs = np.arange(lo, hi+1e-9, sel.band_step_m)
    ys = sign*np.arange(cfg.width_m/2+sel.band_offset_m, cfg.width_m/2+sel.band_width_m+1e-9, sel.band_step_m)
    q = np.stack(np.meshgrid(xs, ys, indexing='ij'), axis=-1).reshape(-1, 2)
    _, l = frame.project(q)
    q = q[np.abs(l) < cfg.wall_offset_m-cfg.wall_tolerance_m]
    if not len(q):
        return {'free': False, 'samples': 0, 'observed_free': 0, 'masked': 0, 'unobserved': 0}
    pad = mx.ray_padding(scene, cfg, .5)
    free = depth_support(scene, q, pad)
    masked = np.zeros(len(q), dtype=bool)
    if np.any(~free):
        masked[~free] = mx.masked_only(scene, q[~free], pad)
    unobserved = ~free & ~masked
    return {'free': bool(np.any(free)) and not bool(np.any(unobserved)), 'samples': int(len(q)), 'observed_free': int(free.sum()),
            'masked': int(masked.sum()), 'unobserved': int(unobserved.sum())}


def extrema_arc(steering, distance, cfg):
    """1 cm 진행 자세에 차체 모서리의 x/y 극점 자세를 합친다. 연속 sweep 인증은 아니다."""
    poses = command_arc(steering, distance, cfg)
    k = math.tan(steering)/cfg.wheelbase_m
    if abs(k) < 1e-9:
        return poses
    end = distance*k
    lo, hi = sorted((0., end))
    angles = list(poses[:, 2])
    for x in (-cfg.length_m/2, cfg.length_m/2):
        for y in (-cfg.width_m/2, cfg.width_m/2):
            a, b = 1/k-y, cfg.wheelbase_m/2+x
            for critical in (math.atan2(a,b), math.atan2(-b,a)):
                angles.extend(critical+n*math.pi for n in range(
                    math.ceil((lo-critical)/math.pi), math.floor((hi-critical)/math.pi)+1))
    theta = np.array(sorted(set(angles), key=lambda v:v/k))
    return np.column_stack((np.sin(theta)/k-cfg.wheelbase_m/2+cfg.wheelbase_m/2*np.cos(theta),
                            (1-np.cos(theta))/k+cfg.wheelbase_m/2*np.sin(theta),theta))


def motion_queries(scene, speed, target, steering, previous, dt, follow_cfg, lat_cfg):
    """현재/직전 조향의 반응+제동 원호에서 현재 차체 밖의 외곽 표본."""
    age = scene.t_ref_s-scene.source_first_s
    v = max(abs(speed), abs(target))
    distance = mx.envelope_distance(v, age, dt, mx.ExecutionConfig())
    queries = []
    for angle in sorted({float(steering), float(previous)}):
        poses = extrema_arc(angle, max(distance, .01), lat_cfg)
        q, _ = mx.body_queries(scene, poses, np.full(len(poses), angle), lat_cfg)
        queries.append(q.reshape(-1, 2))
    q = np.unique(np.vstack(queries), axis=0)
    outside = (np.abs(q[:, 0]) > follow_cfg.length_m/2+1e-9) | (np.abs(q[:, 1]) > follow_cfg.width_m/2+1e-9)
    q = q[outside]
    return distance, q


def motion_region(scene, speed, target, steering, previous, dt, follow_cfg, lat_cfg):
    """실제 반환에 근거한 관측 분류. 별도 후보의 위험 수용으로 unobserved를 지우지 않는다."""
    distance, q = motion_queries(scene, speed, target, steering, previous, dt, follow_cfg, lat_cfg)
    result = {'distance_m': float(distance), 'queries': int(len(q)), 'unobserved': 0, 'masked_left': 0,
              'masked_right': 0, 'masked_tail_extent_m':0., 'masked_front_samples':0}
    if not len(q):
        return result
    pad = mx.ray_padding(scene, lat_cfg, .5)
    free = depth_support(scene, q, pad)
    if np.any(~free):
        masked = np.zeros(len(q), dtype=bool)
        masked[~free] = mx.masked_only(scene, q[~free], pad)
        result['unobserved'] = int(np.sum(~free & ~masked))
        result['masked_left'] = int(np.sum(masked & (q[:, 1] > 0)))
        result['masked_right'] = int(np.sum(masked & (q[:, 1] < 0)))
        if np.any(masked):
            tail=q[masked]
            result['masked_tail_extent_m']=float(np.max(np.maximum(
                np.abs(tail[:,1])-follow_cfg.width_m/2, -tail[:,0]-follow_cfg.length_m/2)))
            result['masked_front_samples']=int(np.sum(tail[:,0]>follow_cfg.length_m/2))
    return result


def region_blocked(region, sel, *, bounded_unknown_risk=False):
    """큰 이동 영역의 기존 거절 규칙. 일반 미관측, 1 cm 초과 가림 꼬리 또는 전면 가림이면 거절 이유를 돌려준다."""
    if region['unobserved'] and not bounded_unknown_risk:
        return 'large_motion_region_unobserved'
    blocked = None
    for name, key in (('left', 'masked_left'), ('right', 'masked_right')):
        if region[key] and (region.get('masked_front_samples', 1) > 0 or
                            region.get('masked_tail_extent_m', math.inf) > sel.masked_tail_allowance_m):
            blocked = f'large_motion_masked_toward_{name}'
    return blocked


def side_extent(frame, poses, sign, cfg):
    """기본 경로 좌표에서 차체 네 모서리의 한쪽 방향 최대 횡위치. 표본 자세 사이 연속 극값 인증은 아니다."""
    p = np.asarray(poses, dtype=float).reshape(-1, 3)
    corner = np.array([[x, y] for x in (-cfg.length_m/2, cfg.length_m/2) for y in (-cfg.width_m/2, cfg.width_m/2)])
    c, s = np.cos(p[:, 2])[:, None], np.sin(p[:, 2])[:, None]
    points = np.stack((p[:, :1]+c*corner[None, :, 0]-s*corner[None, :, 1],
                       p[:, 1:2]+s*corner[None, :, 0]+c*corner[None, :, 1]), axis=-1).reshape(-1, 2)
    _, lateral = frame.project(points)
    return float(np.max(sign*lateral))


class PassReleaseSelector:
    def __init__(self, follow_config, config=None):
        self.follow_config = follow_config
        self.config = config or SelectorConfig()
        self.lat = lateral_config(follow_config)
        self.reset()

    def reset(self):
        """명시 세션 재시작 전용. 센서 중단(reset 이벤트)에는 invalidate를 쓴다."""
        self.sides = {name: self._side() for name, _ in SIDES}
        self.epoch = self.last_source = self.last_ref = None
        self.guard_anchor, self.last_choice = {}, None

    @staticmethod
    def _side():
        return {'seen': False, 'present': False, 'clear': 0, 'passed': False, 'vanish': None, 'last_present': None}

    def invalidate(self, reason):
        """시간 연결을 잃었다. 위협을 본 적이 있으면 양쪽을 unknown으로 두고 새 통과 증거를 요구한다."""
        if any(s['seen'] for s in self.sides.values()):
            for side in self.sides.values():
                side.update(seen=True, present=False, clear=0, passed=False, vanish='unknown', last_present=None)
        self.epoch = self.last_source = self.last_ref = None
        self.guard_anchor, self.last_choice = {}, None
        return reason

    @property
    def mode(self):
        seen = [s for s in self.sides.values() if s['seen']]
        if not seen:
            return 'SOLO'
        return 'RELEASED' if all(s['passed'] and not s['present'] for s in seen) else 'GUARD'

    def _hold(self, previous, detail, reason):
        steer = float(np.clip(previous, -self.follow_config.max_steering_rad, self.follow_config.max_steering_rad)) \
            if math.isfinite(previous) else 0.
        return 0., steer, {**detail, 'selector_reason': reason, 'selector_choice': 'hold_fault',
                          'reason': 'pass_release_'+reason, 'behavior':'HOLD'}

    def select(self, scene, baseline, parallel, speed, previous, dt, *, raw_scan=None):
        out = self._select(scene, baseline, parallel, speed, previous, dt, raw_scan=raw_scan)
        self.last_choice = out[2].get('selector_choice')
        return out

    def _guard_curve(self, scene, view, baseline, parallel, speed, previous, dt, age, detail, raw_scan):
        """GUARD 곡선 후보(선택형). 평행 후보가 알려진 직선 적합 실패로 멈췄을 때만 기존 G1 출력을 저속으로 검사한다.

        G1 경로·점유·팽창·조향 변화율·hysteresis는 기존 출력 그대로이며 속도만 min(G1, 0.35 m/s)로 낮춘다. 측정 속도가 더 크면
        감속 완료를 가정하지 않고 큰 쪽으로 예측한다. 위협을 본(해제되지 않은) 쪽마다 현재/직전 조향 원호의 차체 모서리
        경로 횡위치가 GUARD 진입 때보다 1 cm 넘고 현재보다도 증가하면 거절한다(누적 접근 포함). 이미 기준 밖이어도 더
        접근하지 않는 진행/회복은 막지 않는다. 경로 좌표 추정 변화의 영향은 남으므로 실제 이동 영역도 매번 검사한다.
        통과 증거·vanish·GUARD 상태는 바꾸지 않으며 숨은 물체 위험을 지우지 않는다.
        거절하거나 조건 밖이면 None/평행 후보(정지)를 돌려준다.
        """
        b_speed, b_steer, b_detail = baseline
        p_speed, p_steer, p_detail = parallel
        trigger = p_detail.get('reason')
        continuing = trigger == 'parallel_hold_release_wait' and self.last_choice == 'guard_baseline'
        if p_speed > 0 or not (trigger in GUARD_GEOMETRY_REASONS or continuing):
            return None
        if b_detail.get('reason') not in GUARD_BASELINE_REASONS or not b_speed > 0 or speed < 0:
            return None
        target = min(b_speed, self.config.guard_speed_cap_mps)
        v = max(speed, target)
        info = {'trigger': trigger, 'path': 'baseline_g1', 'baseline_reason': b_detail.get('reason'),
                'baseline_speed_mps': b_speed, 'target_speed_mps': target, 'prediction_speed_mps': v,
                'steering_rad': b_steer, 'previous_steering_rad': previous, 'accepted': False}

        def reject(reason):
            return p_speed, p_steer, {**p_detail, **detail, 'selector_choice': 'parallel',
                                      'selector_reason': 'guard_curve_rejected_'+reason,
                                      'selector_guard_candidate': info}
        if view is None:
            return reject('no_baseline_view')
        distance = mx.envelope_distance(v, age, dt, mx.ExecutionConfig())
        poses = np.vstack([np.vstack((command_arc(a, max(distance, .01), self.lat),
                                      extrema_arc(a, max(distance, .01), self.lat)))
                           for a in sorted({float(b_steer), float(previous)})])
        info.update(envelope_distance_m=distance, guarded_sides={})
        for name, sign in SIDES:
            side = self.sides[name]
            if not side['seen'] or (side['passed'] and not side['present']):
                continue
            current = side_extent(view['frame'], [[0., 0., 0.]], sign, self.lat)
            predicted = side_extent(view['frame'], poses, sign, self.lat)
            anchor = self.guard_anchor.get(name, current)
            info['guarded_sides'][name] = {'anchor_m': anchor, 'current_m': current, 'predicted_m': predicted,
                                           'increase_over_anchor_m': predicted-anchor,
                                           'additional_approach_m': predicted-current,
                                           'not_increasing_present_extent': predicted <= current+1e-9}
            if predicted > anchor+self.config.small_excursion_m and predicted > current+1e-9:
                return reject(f'toward_{name}')
        excursion = lateral_excursion(v, age, b_steer, previous, dt, self.follow_config)
        info['excursion_m'] = excursion
        # GUARD에는 미해제 물체가 있다. 작은 움직임도 현재 관측/마스크 분류를 생략하지 않는다.
        region = motion_region(scene, speed, target, b_steer, previous, dt, self.follow_config, self.lat)
        info['region'] = region
        fringe = {'accepted': False, 'reason': 'not_needed', 'unknown_filled': False}
        if region['unobserved'] and raw_scan is not None:
            _, queries = motion_queries(scene, speed, target, b_steer, previous, dt, self.follow_config, self.lat)
            pad = mx.ray_padding(scene, self.lat, .5)
            free = depth_support(scene, queries, pad)
            masked = np.zeros(len(queries), dtype=bool)
            masked[~free] = mx.masked_only(scene, queries[~free], pad)
            fringe = bounded_rear_fringe(scene, raw_scan, queries[~free & ~masked], pad,
                                         self.follow_config.length_m, self.follow_config.width_m)
        info['rear_fringe'] = fringe
        blocked = region_blocked(region, self.config, bounded_unknown_risk=fringe['accepted'])
        if blocked is not None:
            return reject(blocked)
        small_tail = bool(region['masked_left'] or region['masked_right'])
        info['accepted'] = True
        reason = ('guard_curve_bounded_rear_fringe' if fringe['accepted'] else
                  'guard_curve_observed_motion_small_masked_tail' if small_tail else 'guard_curve_observed_motion')
        return target, b_steer, {**b_detail, **detail, 'selector_choice': 'guard_baseline', 'selector_reason': reason,
                                 'selector_masked_tail_risk_accepted': small_tail,
                                 'selector_bounded_rear_fringe_risk_accepted': fringe['accepted'],
                                 'selector_guard_candidate': info}

    def _select(self, scene, baseline, parallel, speed, previous, dt, *, raw_scan=None):
        detail = {'selector_mode_before': self.mode, 'pass_release_trial':True,
                  'selector_unknown_filled':False, 'selector_hidden_rear_risk_unresolved':True}
        # 두 후보를 모두 검사한다. 미선택 후보의 잘못된 출력도 정상 후보로 숨기지 않는다.
        c = self.follow_config
        motion_valid = (all(math.isfinite(v) for v in (speed, previous, dt))
                        and 0. < dt <= .2 and abs(previous) <= c.max_steering_rad+1e-9)
        outputs_valid = all(math.isfinite(o[0]) and math.isfinite(o[1])
                            and 0. <= o[0] <= c.max_speed_mps+1e-9
                            and abs(o[1]) <= c.max_steering_rad+1e-9 for o in (baseline, parallel))
        if not motion_valid or not outputs_valid:
            return self._hold(previous, detail, self.invalidate('invalid_motion_or_output'))
        source, now = scene.source_first_s, scene.t_ref_s
        if source is None or not math.isfinite(source) or not math.isfinite(now):
            return self._hold(previous, detail, self.invalidate('invalid_time'))
        if ((self.epoch is not None and scene.source_epoch != self.epoch) or
                (self.last_ref is not None and now < self.last_ref) or
                (self.last_source is not None and source < self.last_source)):
            return self._hold(previous, detail, self.invalidate('time_or_epoch_discontinuity'))
        age = now-source
        if not 0 <= age < self.config.max_scene_age_s:
            return self._hold(previous, detail, self.invalidate('scene_age_out_of_range'))
        fresh = self.last_source is None or source > self.last_source
        self.epoch, self.last_source, self.last_ref = scene.source_epoch, source, now
        b_speed, b_steer, b_detail = baseline
        view = threat_view(scene, b_detail.get('path_points'), self.lat)
        evidence = {'fresh': fresh}
        if view is not None and fresh:
            for name, sign in SIDES:
                side, now_side = self.sides[name], view[name]
                evidence[name] = dict(now_side)
                if now_side['present']:
                    side.update(seen=True, present=True, clear=0, passed=False,
                                last_present={'t': now, 'rearmost_ds_m': now_side['rearmost_ds_m'],
                                              'frontmost_ds_m': now_side['frontmost_ds_m']})
                    if side['vanish'] == 'benign':
                        side['vanish'] = None
                    continue
                if not side['seen']:
                    continue
                if side['present']:
                    rear = (side['last_present'] or {}).get('rearmost_ds_m')
                    front = self.follow_config.length_m/2+self.config.pass_front_margin_m
                    vanish = 'benign' if rear is not None and rear >= front else 'rear'
                    if side['vanish'] not in ('rear', 'unknown'):
                        side['vanish'] = vanish
                    side['present'] = False
                band = band_check(scene, view['frame'], sign, self.lat, self.config)
                evidence[name]['band'] = band
                clear = band['free'] and side['vanish'] == 'benign'
                if side['vanish'] in ('rear', 'unknown'):
                    # 띠만으로는 해제하지 않는다. 측방 위협 창 전체가 현재 관측되어야 한다(기록은 유지).
                    window = band_check(scene, view['frame'], sign, self.lat, self.config,
                                        (-self.lat.band_behind_m, self.lat.band_ahead_m))
                    evidence[name]['side_window'] = window
                    clear = window['free']
                side['clear'] = side['clear']+1 if clear else 0
                side['passed'] = side['vanish'] in ('benign', 'rear', 'unknown') and \
                    side['clear'] >= self.config.release_fresh
        elif fresh and view is None:
            for side in self.sides.values():
                if side['seen'] and not side['present']:
                    side['clear'] = 0
                    side['passed'] = False
            evidence['view'] = 'no_baseline_path'
        mode = self.mode
        if mode != 'GUARD':
            self.guard_anchor = {}
        elif c.guard_curve_trial and view is not None:
            # GUARD 진입 때(첫 관측 경로) 해제되지 않은 쪽의 차체 모서리 경로 횡위치. 누적 접근의 기준이다.
            for name, sign in SIDES:
                side = self.sides[name]
                if name not in self.guard_anchor and side['seen'] and not (side['passed'] and not side['present']):
                    self.guard_anchor[name] = side_extent(view['frame'], [[0., 0., 0.]], sign, self.lat)
        detail.update(selector_mode=mode, selector_evidence=evidence,
                      selector_sides={k: {kk: v[kk] for kk in ('seen', 'present', 'clear', 'passed', 'vanish')}
                                      for k, v in self.sides.items()})
        if speed < 0.:
            # 제동 중 유한 음수 모터축 속도는 전진 출력을 막지만 센서 시계 단절은 아니다.
            # 같은 시각 계열의 관측 기억은 유지하며 속도를 0으로 바꿔 관측을 위조하지 않는다.
            if mode == 'SOLO' and b_speed <= 0.:
                # 위협을 본 적이 없으면 기존 G1의 역행 정지(조향 포함)를 바꾸지 않는다(OFF 판단 동일성).
                return b_speed, b_steer, {**b_detail, **detail, 'selector_choice': 'baseline',
                                          'selector_reason': 'solo_reverse_baseline_stop'}
            return self._hold(previous, detail, 'reverse_motion_hold')
        if b_detail.get('reason') not in NORMAL_BASELINE:
            # 기존 G1 자체의 정지·입력 거절. 증거는 위에서 갱신했고, 출력은 정지 쪽만 따른다(양수 금지).
            chosen = baseline if mode == 'SOLO' else parallel
            if chosen[0] > 0:
                return self._hold(previous, detail, 'baseline_not_normal_positive_blocked')
            return chosen[0], chosen[1], {**chosen[2], **detail, 'selector_reason': 'baseline_not_normal',
                                          'selector_choice': 'baseline' if mode == 'SOLO' else 'parallel'}
        if mode == 'SOLO':
            return b_speed, b_steer, {**b_detail, **detail, 'selector_choice': 'baseline',
                                      'selector_reason': 'no_side_threat_seen'}
        if mode == 'GUARD':
            p_speed, p_steer, p_detail = parallel
            if c.guard_curve_trial:
                guarded = self._guard_curve(scene, view, baseline, parallel, speed, previous, dt, age, detail, raw_scan)
                if guarded is not None:
                    return guarded
            return p_speed, p_steer, {**p_detail, **detail, 'selector_choice': 'parallel',
                                      'selector_reason': 'side_threat_not_passed'}
        if b_speed <= 0:
            # 출발이 승인되기 전에 중앙 복귀각으로 미리 돌리면 fallback의 직선 조건과 교착한다.
            # 정지 명령은 그대로 두고 검증되지 않은 복귀 조향만 직전 발행각으로 유지한다.
            return b_speed, previous, {**b_detail, **detail, 'selector_choice': 'baseline',
                                       'selector_reason': 'released_hold_keep_steering',
                                       'selector_held_baseline_steering_rad':b_steer}
        excursion = lateral_excursion(max(speed, b_speed), age, b_steer, previous, dt, self.follow_config)
        detail['selector_excursion_m'] = excursion
        if excursion <= self.config.small_excursion_m:
            return b_speed, b_steer, {**b_detail, **detail, 'selector_choice': 'baseline',
                                      'selector_reason': 'released_small_motion'}
        region = motion_region(scene, speed, b_speed, b_steer, previous, dt, self.follow_config, self.lat)
        detail['selector_region'] = region
        blocked = region_blocked(region, self.config)
        if blocked is None:
            small_tail=bool(region['masked_left'] or region['masked_right'])
            return b_speed, b_steer, {**b_detail, **detail, 'selector_choice': 'baseline',
                'selector_masked_tail_risk_accepted':small_tail,
                'selector_reason':'released_observed_motion_small_masked_tail' if small_tail
                                  else 'released_large_motion_observed'}
        p_speed, p_steer, p_detail = parallel
        return p_speed, p_steer, {**p_detail, **detail, 'selector_choice': 'parallel', 'selector_reason': blocked}

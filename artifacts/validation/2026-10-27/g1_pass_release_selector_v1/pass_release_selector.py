"""옆차 통과 뒤 기존 G1 출력을 허용할지 고르는 상태 선택 시제품. 운영 노드·launch·기본값에 연결하지 않는다.

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
   바꾸지 않고 따로 센다. `benign`인 쪽만 통과를 승인하며 `rear`/`unknown`은 빈 띠만으로 해제하지 않는다.

RELEASED의 기존 G1 출력 허용
- 속도 0이면 그대로(HOLD).
- 현재/직전 조향의 반응+제동 구간 차체 모서리 횡변위가 `small_excursion_m` 이하이면 허용한다. 이는 기존 평행 후보와 같은
  한계이며, 후방 마스크의 뒤차·꼬리 띠 미세 물체 위험은 해소하지 않은 채 남긴다(ADR 0027의 작은 이동 재검토 범위).
- 더 크면 두 조향의 정곡률 원호 차체·바퀴 외곽 중 현재 차체 밖 표본을 현재 스캔으로 검사한다. 차체 모서리의 x/y
  극점 자세를 추가하고 미관측·마스크 표본이 있으면 거부한다. 통과 기록을 현재 자유 관측으로 바꾸지 않는다.
  거부되면 평행 후보 출력을 쓴다. 표본 자세 사이 연속 영역과 숨은 물체의 미래 접근은 여전히 미입증이다.

고장: 시각 역행·세대 변경·장면 나이 0.3 s 이상·기존 G1 입력 정지면 이번 판단은 HOLD(양수 금지)이다. 위협을 본 적이 있으면
양쪽을 `unknown`으로 표시하고 새 통과 증거를 다시 요구한다. 해제 뒤 같은 쪽 위협이 다시 보이면 GUARD로 돌아간다.
"""
from dataclasses import dataclass
import math

import numpy as np

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.lateral_intent import (LateralIntentConfig, ObservationGrid, cell_coordinates, side_threats)
from arena_autonomy.m3_observed_follow import command_arc
from arena_autonomy.observation_support import depth_support
from arena_autonomy.parallel_lane_trial import lateral_excursion

NORMAL_BASELINE = ('local_follow', 'follow_gap_hold', 'follow_release_wait', 'local_follow_corridor_candidate')
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

    def __post_init__(self):
        if type(self.release_fresh) is not int or self.release_fresh < 2:
            raise ValueError('release requires at least two distinct scans')
        for key, value in vars(self).items():
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('selector constants must be finite and positive')
        if self.band_offset_m >= self.band_width_m or self.max_scene_age_s > .3:
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


def band_check(scene, frame, sign, cfg, sel):
    """옆면 밖 띠의 양의 자유 관측. 마스크 표본은 자유로 세지 않고 따로 센다."""
    xs = np.arange(-cfg.length_m/2, cfg.length_m/2+1e-9, sel.band_step_m)
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


def motion_region(scene, speed, target, steering, previous, dt, follow_cfg, lat_cfg):
    """현재/직전 조향의 반응+제동 원호 차체·바퀴 외곽 중 현재 차체 밖 표본의 관측 분류."""
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
    result = {'distance_m': float(distance), 'queries': int(len(q)), 'unobserved': 0, 'masked_left': 0,
              'masked_right': 0}
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
    return result


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

    @staticmethod
    def _side():
        return {'seen': False, 'present': False, 'clear': 0, 'passed': False, 'vanish': None, 'last_present': None}

    def invalidate(self, reason):
        """시간 연결을 잃었다. 위협을 본 적이 있으면 양쪽을 unknown으로 두고 새 통과 증거를 요구한다."""
        if any(s['seen'] for s in self.sides.values()):
            for side in self.sides.values():
                side.update(seen=True, present=False, clear=0, passed=False, vanish='unknown', last_present=None)
        self.epoch = self.last_source = self.last_ref = None
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
        return 0., steer, {**detail, 'selector_reason': reason, 'selector_choice': 'hold_fault'}

    def select(self, scene, baseline, parallel, speed, previous, dt):
        detail = {'selector_mode_before': self.mode}
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
                side['clear'] = side['clear']+1 if band['free'] else 0
                side['passed'] = side['vanish'] == 'benign' and side['clear'] >= self.config.release_fresh
        elif fresh and view is None:
            for side in self.sides.values():
                if side['seen'] and not side['present']:
                    side['clear'] = 0
                    side['passed'] = False
            evidence['view'] = 'no_baseline_path'
        mode = self.mode
        detail.update(selector_mode=mode, selector_evidence=evidence,
                      selector_sides={k: {kk: v[kk] for kk in ('seen', 'present', 'clear', 'passed', 'vanish')}
                                      for k, v in self.sides.items()})
        if speed < 0.:
            # 제동 중 유한 음수 모터축 속도는 전진 출력을 막지만 센서 시계 단절은 아니다.
            # 같은 시각 계열의 관측 기억은 유지하며 속도를 0으로 바꿔 관측을 위조하지 않는다.
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
            return p_speed, p_steer, {**p_detail, **detail, 'selector_choice': 'parallel',
                                      'selector_reason': 'side_threat_not_passed'}
        if b_speed <= 0:
            return b_speed, b_steer, {**b_detail, **detail, 'selector_choice': 'baseline',
                                      'selector_reason': 'released_baseline_hold'}
        excursion = lateral_excursion(max(speed, b_speed), age, b_steer, previous, dt, self.follow_config)
        detail['selector_excursion_m'] = excursion
        if excursion <= self.config.small_excursion_m:
            return b_speed, b_steer, {**b_detail, **detail, 'selector_choice': 'baseline',
                                      'selector_reason': 'released_small_motion'}
        region = motion_region(scene, speed, b_speed, b_steer, previous, dt, self.follow_config, self.lat)
        detail['selector_region'] = region
        blocked = None
        if region['unobserved']:
            blocked = 'large_motion_region_unobserved'
        else:
            for name, key in (('left', 'masked_left'), ('right', 'masked_right')):
                if region[key]:
                    blocked = f'large_motion_masked_toward_{name}'
        if blocked is None:
            return b_speed, b_steer, {**b_detail, **detail, 'selector_choice': 'baseline',
                                      'selector_reason': 'released_large_motion_observed'}
        p_speed, p_steer, p_detail = parallel
        return p_speed, p_steer, {**p_detail, **detail, 'selector_choice': 'parallel', 'selector_reason': blocked}

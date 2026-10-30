"""기본 OFF 선택형 observed_fresh_v2: 현재 정지 검사와 fresh 관측 확인 기억을 분리한다.

현재 적격성·팽창·0명령은 observed_measured_v1과 같고, 같은 스캔 재사용에서
나이만 늘어 거절된 경우 이전 확인 1회를 제한적으로 남긴다. 같은 광선의 강체
변환 일치, 차체 네 모서리 이동 5 mm, 조향 0.005 rad, 0.15 s를 요구한다.
원거리 점 이동을 차체 이동으로 오해하지 않는다. 새 fresh 장애물/시간 단절/
비나이 거절은 확인을 지운다. 후방 미관측을 자유 공간으로 만들지 않는다.
"""
import math

import numpy as np

from arena_autonomy.parallel_lane_trial import ParallelLaneTrial
from arena_autonomy.fresh_parallel_speed_trial import proposal

GEOMETRY_POINT_TOLERANCE_M = .005
GEOMETRY_STEERING_TOLERANCE_RAD = .005
MEMORY_WINDOW_S = .15


RIGID_NUMERICAL_TOLERANCE = 1e-7
BODY_MOTION_LIMIT_M = GEOMETRY_POINT_TOLERANCE_M


def snapshot(scene):
    rays = scene.rays
    return dict(
        signature=(scene.source_frame_id, scene.source_epoch, scene.source_first_s, scene.source_last_s,
                   tuple((r.source_index, r.acquired_at_s, r.status, r.source_frame_id, r.source_epoch) for r in rays)),
        origins=np.asarray([r.ray_origin_xy_m for r in rays], dtype=float).reshape(-1, 2),
        directions=np.asarray([r.direction_xy for r in rays], dtype=float).reshape(-1, 2),
        points=np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2))


def geometry_check(current, old, length, width):
    if current['signature'] != old['signature'] or not len(current['origins']):
        return dict(same_raw_scan=False, rigid_scan=False, small_body_motion=False)
    a, b = old['directions'][0], current['directions'][0]
    c, s = float(a@b), float(a[0]*b[1]-a[1]*b[0])
    rotation = np.array([[c, s], [-s, c]])
    translation = current['origins'][0]-old['origins'][0]@rotation
    residuals = []
    for key in ('origins', 'points', 'directions'):
        q, p = old[key], current[key]
        if q.shape != p.shape:
            return dict(same_raw_scan=True, rigid_scan=False, small_body_motion=False)
        if len(p):
            prediction = q@rotation+(0. if key == 'directions' else translation)
            residuals.append(float(np.linalg.norm(prediction-p, axis=1).max()))
    residual = max(residuals, default=float('inf'))
    corners = np.array([[x*length/2, y*width/2] for x in (-1, 1) for y in (-1, 1)])
    motion = float(np.linalg.norm(corners@rotation+translation-corners, axis=1).max())
    return dict(same_raw_scan=True, rigid_scan=residual <= RIGID_NUMERICAL_TOLERANCE,
                small_body_motion=motion <= BODY_MOTION_LIMIT_M,
                rigid_residual=residual, body_corner_displacement_m=motion)


class FreshMemoryParallelTrial(ParallelLaneTrial):
    def reset(self):
        super().reset()
        self.confirmed = None
        self.remembered = False

    def _record(self, scene, baseline, previous, result, now):
        return dict(source=scene.source_first_s, t_ref=now, scene_snapshot=snapshot(scene),
                    wall=baseline[2].get('preferred_wall'), steering=result['steering_rad'], previous=previous)

    def _classify(self, scene, baseline, speed, previous, dt, result, now):
        info = dict(kept=False, rejection=result['reason'] if not result['eligible'] else 'below_resume_clearance',
                    current_clearance_m=result.get('clearance_m'))
        record = self.confirmed
        if record is None:
            info['why'] = 'no_valid_fresh_confirmation_for_this_scan'
            return info
        geometry = geometry_check(snapshot(scene), record['scene_snapshot'], self.config.length_m, self.config.width_m)
        steering = result.get('steering_rad')
        same_steering = (steering is not None and math.isfinite(previous)
                         and abs(steering-record['steering']) <= GEOMETRY_STEERING_TOLERANCE_RAD
                         and abs(previous-record['previous']) <= GEOMETRY_STEERING_TOLERANCE_RAD)
        checks = {k: geometry[k] for k in ('same_raw_scan', 'rigid_scan', 'small_body_motion')}
        checks.update(same_wall=baseline[2].get('preferred_wall') == record['wall'],
                      within_window=0 <= now-record['t_ref'] <= MEMORY_WINDOW_S,
                      occupancy_gap_rejection=result['reason'] in ('unchanged_gap_hold', 'parallel_current_position'),
                      same_steering=same_steering)
        info.update(confirmed_t_ref=record['t_ref'], elapsed_since_confirmation_s=now-record['t_ref'],
                    body_geometry=geometry)
        if all(checks.values()):
            counterfactual = proposal(scene, baseline, speed, previous, dt, self.config,
                                      padding_reference_s=record['t_ref'])
            checks['counterfactual_clears'] = bool(counterfactual['eligible']
                                                   and counterfactual['clearance_m'] >= self.config.resume_clearance_m)
            info.update(counterfactual_reason=counterfactual['reason'],
                        counterfactual_clearance_m=counterfactual.get('clearance_m'))
        info['checks'] = checks
        info['kept'] = all(checks.values()) and 'counterfactual_clears' in checks
        return info

    def apply(self, scene, baseline, speed, previous, dt):
        source, now = scene.source_first_s, scene.t_ref_s
        hold_steer = float(np.clip(previous, -self.config.max_steering_rad,
                                   self.config.max_steering_rad)) if math.isfinite(previous) else 0.
        if source is None or not math.isfinite(source) or not math.isfinite(now):
            self.reset()
            return 0., hold_steer, {**baseline[2], 'reason': 'parallel_hold_invalid_time',
                                    'behavior': 'HOLD', 'parallel_hold_selected': False}
        if (self.epoch != scene.source_epoch or
                (self.last_ref is not None and now < self.last_ref) or
                (self.last_source is not None and source < self.last_source)):
            self.reset()
        fresh = self.last_source is None or source > self.last_source
        self.epoch, self.last_source, self.last_ref = scene.source_epoch, source, now
        if fresh and self.remembered:
            # 기억은 다음 fresh 평가 한 번에만 쓰인다. 스캔 누락(간격 초과)이면 평가 전에 지운다.
            if self.confirmed is None or source-self.confirmed['source'] > MEMORY_WINDOW_S:
                self.confirmations = 0
            self.remembered = False
        result = proposal(scene, baseline, speed, previous, dt, self.config)
        detail = {**baseline[2], 'baseline_reason': baseline[2].get('reason'),
                  'parallel_hold_trial': result, 'parallel_hold_selected': False,
                  'reason': 'parallel_hold_'+result['reason'], 'behavior': 'HOLD'}
        resume = self.config.resume_clearance_m
        below_resume = result['eligible'] and self.held and result['clearance_m'] < resume
        memory = None
        if not fresh and (not result['eligible'] or below_resume):
            memory = self._classify(scene, baseline, speed, previous, dt, result, now)
            detail['fresh_confirmation_memory'] = memory
        kept = bool(memory and memory['kept'])
        if fresh:
            self.confirmed = (self._record(scene, baseline, previous, result, now)
                              if result['eligible'] and result['clearance_m'] >= resume else None)
        elif (not result['eligible'] or below_resume) and not kept:
            self.confirmed = None
        if not result['eligible']:
            self.held = True
            self.confirmations, self.remembered = (1, True) if kept else (0, False)
            return 0., hold_steer, detail
        steering = result['steering_rad']
        if self.held:
            if result['clearance_m'] < resume:
                self.confirmations, self.remembered = (1, True) if kept else (0, False)
            elif fresh:
                self.confirmations += 1
            if self.confirmations < self.config.confirmations:
                return 0., steering, {**detail, 'reason': 'parallel_hold_release_wait',
                                      'parallel_release_count': self.confirmations}
            self.held = False
        detail.update(parallel_hold_selected=True, reason='parallel_hold_follow', behavior='FOLLOW',
                      path_points=result['path_points'], swept_clearance_m=result['clearance_m'],
                      target_speed_before_hold_mps=result['target_speed_mps'])
        return result['target_speed_mps'], steering, detail

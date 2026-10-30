"""오프라인 상태 후보(운영 미연결): fresh 재개 확인 기억과 '같은 스캔에서 시간만 늘어 막힌 주기'를 분리한다.

런타임 MeasuredParallelTrial(observed_measured_v1)과 다른 점은 재개 확인 수(confirmations)를 지우는지 하나뿐이다.
- 매 호출의 적격성·명령·임계값(hold/resume·팽창·확인 2회)은 런타임과 같은 proposal 계산이다. 거절이면 반드시 0을 낸다.
- fresh 스캔이 적격이고 resume 여유 이상이면 그 스캔을 '확인 스캔'으로 기록한다(원래 시각·raw 광선 서명·점·조향·벽).
- 그 스캔을 재사용하는 호출이 점유 여유 때문에 거절(또는 HOLD 중 resume 미만)될 때, 아래를 모두 만족해야만 이 스캔의
  확인 1회를 남긴다: 같은 epoch/source/raw 광선 취득 시각, 같은 벽 기준, 점 이동 <= 5 mm, 현재/직전 조향 변화 <= 0.005 rad,
  확인 시각부터 0.15 s 이내, 그리고 같은 입력에서 팽창 나이만 확인 시각 기준으로 되돌린 반사실이 적격·resume 이상.
- 기억은 확인 수를 늘리지 않으며 확인 시각을 갱신하지 않는다(TTL 연장 없음). 다음 fresh 스캔은 평소처럼 평가하고
  거절되면 즉시 지운다. 확인 스캔과 다음 fresh 스캔 간격이 0.15 s를 넘으면 평가 전에 지운다.
- 같은 스캔에서 위 조건을 못 갖춘 거절이 한 번이라도 나면 그 스캔의 확인 기록을 버린다(이후 복원 금지).
- 시간 역행·epoch 변경·잘못된 시각·reset은 런타임과 같이 전부 지운다.
허용치 5 mm/0.005 rad/0.15 s는 이 오프라인 후보의 분류 기준이며 측정 정밀도나 운영 임계값이 아니다.
"""
import math

import numpy as np

from arena_autonomy.parallel_lane_trial import ParallelLaneTrial
from candidate_speed import proposal

GEOMETRY_POINT_TOLERANCE_M = .005
GEOMETRY_STEERING_TOLERANCE_RAD = .005
MEMORY_WINDOW_S = .15


def raw_signature(scene):
    """같은 raw 취득인지 확인할 근거: epoch·source 시작 시각·반환 광선 원래 취득 시각 전부."""
    return (scene.source_epoch, scene.source_first_s,
            tuple(float(r.acquired_at_s) for r in scene.rays if r.status == 'return'))


class FreshMemoryParallelTrial(ParallelLaneTrial):
    def reset(self):
        super().reset()
        self.confirmed = None
        self.remembered = False

    def _record(self, scene, baseline, previous, result, now):
        return dict(source=scene.source_first_s, t_ref=now, signature=raw_signature(scene),
                    wall=baseline[2].get('preferred_wall'), steering=result['steering_rad'], previous=previous,
                    points=np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2).copy())

    def _classify(self, scene, baseline, speed, previous, dt, result, now):
        info = dict(kept=False, rejection=result['reason'] if not result['eligible'] else 'below_resume_clearance',
                    current_clearance_m=result.get('clearance_m'))
        record = self.confirmed
        if record is None:
            info['why'] = 'no_valid_fresh_confirmation_for_this_scan'
            return info
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
        same_points = (points.shape == record['points'].shape and len(points) > 0
                       and float(np.max(np.abs(points-record['points']))) <= GEOMETRY_POINT_TOLERANCE_M)
        steering = result.get('steering_rad')
        same_steering = (steering is not None and math.isfinite(previous)
                         and abs(steering-record['steering']) <= GEOMETRY_STEERING_TOLERANCE_RAD
                         and abs(previous-record['previous']) <= GEOMETRY_STEERING_TOLERANCE_RAD)
        checks = dict(same_raw_scan=raw_signature(scene) == record['signature'],
                      same_wall=baseline[2].get('preferred_wall') == record['wall'],
                      within_window=0 <= now-record['t_ref'] <= MEMORY_WINDOW_S,
                      occupancy_gap_rejection=result['reason'] in ('unchanged_gap_hold', 'parallel_current_position'),
                      same_points=same_points, same_steering=same_steering)
        info.update(confirmed_t_ref=record['t_ref'], elapsed_since_confirmation_s=now-record['t_ref'])
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

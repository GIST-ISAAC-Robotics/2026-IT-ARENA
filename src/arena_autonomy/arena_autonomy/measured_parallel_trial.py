"""선택형 시험, 기본 미사용: observed_measured_v1의 평행 후보. 기존 평행후보의 시간/재개 상태 처리는 그대로다."""
import math
import numpy as np
from arena_autonomy.parallel_lane_trial import ParallelLaneTrial
from arena_autonomy.measured_parallel_speed_trial import proposal

class MeasuredParallelTrial(ParallelLaneTrial):
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
        result = proposal(scene, baseline, speed, previous, dt, self.config)
        detail = {**baseline[2], 'baseline_reason': baseline[2].get('reason'),
                  'parallel_hold_trial': result, 'parallel_hold_selected': False,
                  'reason': 'parallel_hold_'+result['reason'], 'behavior': 'HOLD'}
        if not result['eligible']:
            self.held, self.confirmations = True, 0
            return 0., hold_steer, detail
        steering = result['steering_rad']
        if self.held:
            if result['clearance_m'] < self.config.resume_clearance_m:
                self.confirmations = 0
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

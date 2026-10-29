"""저장 입력 전용 정책 사본. 운영 모듈을 수정하거나 ROS에 연결하지 않는다."""
import inspect
import sys
import types

import numpy as np

from arena_autonomy import partial_observation as original
from arena_autonomy.scene_motion import relative_poses
from probe_timed_missing_sweep import timed_missing_sweep


def policy_class():
    source = inspect.getsource(original)
    before = 'threatening = (along >= 0) & (along <= required) & (across <= spread) & relevant[None,:]'
    after = '''threatening = self._timed_threat(carried, previous, origins, direction,
                source + unknown*scan.time_increment, now, step/2, required, speed,
                now-source, steering, previous_steering, wheelbase, length, width, report)'''
    assert source.count(before) == 1
    changed = source.replace(before, after)
    module = types.ModuleType('timed_partial_observation_diagnostic')
    sys.modules[module.__name__] = module
    exec(compile(changed, original.__file__, 'exec'), module.__dict__)

    class TimedPolicy(module.PartialObservationPolicy):
        def reset(self):
            super().reset()
            self.point_times = {}
            self.current_times = None

        def evaluate(self, scan, points, history, now, speed, steering=0., previous_steering=0.,
                     wheelbase=.135, length=.20, width=.15):
            first = scan.header.stamp.sec + scan.header.stamp.nanosec*1e-9
            expected, ids, _ = history.project(scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max, first, scan.time_increment, now,
                mode='both', timing_verified=True)
            points = np.asarray(points, dtype=float).reshape(-1,2)
            if expected.shape != points.shape or not np.allclose(expected, points, rtol=0., atol=1e-8):
                raise ValueError('point order/coordinate mismatch in timed diagnostic')
            self.current_times = first + ids*scan.time_increment
            return super().evaluate(scan, points, history, now, speed, steering,
                                    previous_steering, wheelbase, length, width)

        def _remember(self, points, now, source, fresh):
            if fresh:
                p = np.asarray(points, dtype=float).reshape(-1,2)
                keep = np.all(np.isfinite(p),axis=1) & (np.linalg.norm(p,axis=1) <= 3.)
                if self.current_times is None or len(self.current_times) != len(p):
                    raise ValueError('acquisition metadata missing')
                self.point_times[(source,now)] = self.current_times[keep].copy()
            super()._remember(points,now,source,fresh)
            keys = {(s[2],s[1]) for s in (self.previous,self.previous_distinct) if s is not None}
            self.point_times = {k:v for k,v in self.point_times.items() if k in keys}

        def _timed_threat(self, carried, previous, origins, directions, stamps, now,
                         half_angle, required, speed, age, steering, previous_steering,
                         wheelbase, length, width, report):
            times = self.point_times.get((previous[2],previous[1]))
            if times is None or len(times) != len(carried):
                raise ValueError('previous acquisition metadata missing')
            result = timed_missing_sweep(carried,times,origins,directions,stamps,now,
                half_angle,required,speed,age,steering,previous_steering,
                wheelbase=wheelbase,length=length,width=width)
            distances = np.asarray(result['lower_distance_m'])
            radii = np.asarray(result['before_radius_m'])
            threat = distances <= radii + 1e-10
            report['timed_missing_diagnostic'] = dict(
                points=len(carried), missing_rays=len(stamps), possible_pairs=int(threat.sum()),
                oldest_point_s=float(np.min(times)), checked_speed_mps=speed,
                checked_steering_rad=steering, checked_previous_steering_rad=previous_steering,
                checked_age_s=age, reference_s=now, source_first_s=now-age,
                required_m=required, wheelbase_m=wheelbase,length_m=length,width_m=width)
            return threat

    return TimedPolicy, source, changed

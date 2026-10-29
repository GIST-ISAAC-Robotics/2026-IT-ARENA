"""명시 선택형 교통 회복 정책. 기본 OFF; 계획기와 독립 보호층은 각 객체를 가진다.

기존 2 m/s 상대 이동·3 cm 여유·표면 대응 조건을 보존한다. 정적 배경 가정에
의존하며 동행 물체의 신원 구별이나 임의 속도의 cut-in 안전을 보장하지 않는다.
"""
import copy
import numpy as np
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.recovery_geometry import timed_missing_sweep
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial

class TimedPolicy(PartialObservationPolicy):
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
            raise ValueError('point order/coordinate mismatch in timed trial')
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

    def _refine_threat(self, threatening, carried, previous, origins, directions, stamps, now,
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

class RecoveryPolicy(TimedPolicy):
    def evaluate(self, *args, **kwargs):
        try:
            return self._evaluate(*args, **kwargs)
        except (ValueError, TypeError, MotionUnavailable) as error:
            # 무결성 실패가 래치 해제가 되지 않도록 기존 기억을 지우지 않는다.
            return dict(allowed=False, reason='traffic_recovery_invalid', speed_cap_mps=0.,
                unknown_filled=False, residual_hidden_object_risk=True, error=str(error))

    def reset(self):
        super().reset()
        self.anchor_probe=SurfaceAnchorTrial()
        self.anchor_latch_time=None

    def _evaluate(self, scan, points, history, now, speed, steering=0., previous_steering=0.,
                 wheelbase=.135, length=.20, width=.15):
        arguments=(scan,points,history,now,speed,steering,previous_steering,wheelbase,length,width)
        result=super().evaluate(*arguments)
        if not self.latched:
            self.anchor_probe.reset();self.anchor_latch_time=None
            return result
        if (result.get('reason')!='known_obstacle_latched' or
                self.latch_reason!='previous_point_in_missing_sweep'):
            return result
        evidence=self.latch_evidence or {}
        witnesses=evidence.get('witnesses',[])
        # 현재 기록은 최대8쌍이다. 잘린 기록을 전체 위협으로 간주하지 않는다.
        if not witnesses or evidence.get('pairs')!=len(witnesses):
            return {**result,'anchor_diagnostic':dict(reason='incomplete_threat_set',released=False)}
        first=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
        projected,ids,_=history.project(scan.ranges,scan.angle_min,scan.angle_increment,
            scan.range_min,scan.range_max,first,scan.time_increment,now,mode='both',timing_verified=True)
        if projected.shape != np.asarray(points).shape or not np.allclose(points,projected,rtol=0,atol=1e-8):
            raise ValueError('recovery coordinate mismatch')
        if self.anchor_latch_time!=self.latch_time:
            self.anchor_probe.start([w['previous_point_now_xy'] for w in witnesses],points,ids,now,first)
            self.anchor_latch_time=self.latch_time
            return {**result,'anchor_diagnostic':dict(reason='initialized',released=False)}
        diagnostic=self.anchor_probe.observe(points,ids,history,now,first)
        result={**result,'anchor_diagnostic':{**diagnostic,'released':False}}
        if not diagnostic['matched']:
            return result
        # 옛 증거가 다시 보였어도 현재 스캔의 다른 위협/공백을 통과해야 한다.
        trial=copy.deepcopy(self)
        trial.latched=False
        trial.latch_reason=trial.latch_time=trial.latch_evidence=None
        reevaluated=TimedPolicy.evaluate(trial,*arguments)
        result['anchor_diagnostic']['current_recheck_reason']=reevaluated.get('reason')
        if not reevaluated.get('allowed'):
            return result
        # 현재 입력 재검사 통과 때만 후보 상태를 채택한다. 같은 스캔을 추가 확인표로 세지 않는다.
        self.__dict__=trial.__dict__
        self.anchor_probe.reset();self.anchor_latch_time=None
        return {**reevaluated,'anchor_diagnostic':{**diagnostic,'released':True,
            'current_recheck_reason':reevaluated.get('reason')}}

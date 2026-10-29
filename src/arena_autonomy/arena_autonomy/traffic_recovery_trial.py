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
from arena_autonomy.anchor_clearance_trial import RollingClearanceTrial
from arena_autonomy.scene_motion import motion_scene

class TimedPolicy(PartialObservationPolicy):
    def reset(self):
        super().reset()
        self.point_times = {}
        self.point_ids = {}
        self.current_times = None
        self.current_ids = None

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
        self.current_ids = ids
        return super().evaluate(scan, points, history, now, speed, steering,
                                previous_steering, wheelbase, length, width)

    def _remember(self, points, now, source, fresh):
        if fresh:
            p = np.asarray(points, dtype=float).reshape(-1,2)
            keep = np.all(np.isfinite(p),axis=1) & (np.linalg.norm(p,axis=1) <= 3.)
            if (self.current_times is None or len(self.current_times) != len(p) or
                    self.current_ids is None or len(self.current_ids) != len(p)):
                raise ValueError('acquisition metadata missing')
            self.point_times[(source,now)] = self.current_times[keep].copy()
            self.point_ids[(source,now)] = self.current_ids[keep].copy()
        super()._remember(points,now,source,fresh)
        keys = {(s[2],s[1]) for s in (self.previous,self.previous_distinct) if s is not None}
        self.point_times = {k:v for k,v in self.point_times.items() if k in keys}
        self.point_ids = {k:v for k,v in self.point_ids.items() if k in keys}

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
    def evaluate(self, scan, *args, **kwargs):
        try:
            source=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            self.assessment_fresh=source!=self.last_source
            self.same_scan_wait=None
            result=self._evaluate(scan, *args, **kwargs)
            if self.same_scan_wait is not None and not self.latched and result.get('allowed'):
                return {**result,'allowed':False,'reason':'same_scan_missing_wait','speed_cap_mps':0.,
                    'same_scan_wait':self.same_scan_wait,'unknown_filled':False,
                    'residual_hidden_object_risk':True}
            return result
        except (ValueError, TypeError, MotionUnavailable) as error:
            # 무결성 실패가 기존 장애물 기억을 지우거나 진행 허가가 되지 않는다.
            return dict(allowed=False, reason='traffic_recovery_invalid', speed_cap_mps=0.,
                unknown_filled=False, residual_hidden_object_risk=True, error=str(error))
        finally:
            # 내부 직접 래치 호출/예외 뒤 문맥은 같은 스캔 예외로 처리하지 않는다.
            self.assessment_fresh=None

    def _latch(self,reason,now):
        if (reason=='previous_point_in_missing_sweep' and self.assessment_fresh is False
                and not self.latched):
            evidence=self.latch_evidence or {}
            self.same_scan_wait=dict(possible_pairs=evidence.get('pairs'),
                source_first_s=self.last_source,reference_s=now,
                fresh_measurement=False,permanent_latch_created=False,
                scope='wait_for_new_scan_not_old_object_absence_proof')
            self.latch_evidence=None
            # 현재 재관측으로 제거되고도 남은 위협을 이번 호출의 0명령 대기로 둔다.
            # 미관측으로 계속 접근하는 물체의 장기 추적/부재 증명은 아니다.
            return
        return super()._latch(reason,now)

    def reset(self):
        super().reset()
        # 명시 선택형 교통 회복에서만 네 점 표면과 작은 정합의 분할 검사를 쓴다.
        # 기본 부분 관측·현재 점군·독립 보호의 장애물 검사는 변경하지 않는다.
        self.anchor_probe=SurfaceAnchorTrial(extended_surface=True,small_alignment=True)
        self.clearance_probe=RollingClearanceTrial()
        self.anchor_latch_time=None
        self.anchor_threat_summary=None
        self.assessment_fresh=None
        self.same_scan_wait=None

    def _capture_latched_threat(self, threatening, previous, now):
        if self.same_scan_wait is not None and not self.latched:return
        old,reference,source=previous
        ids=self.point_ids.get((source,reference))
        matrix=np.asarray(threatening)
        if (matrix.ndim!=2 or matrix.dtype!=np.bool_ or matrix.shape[1]!=len(old)
                or ids is None or len(ids)!=len(old)):
            raise ValueError('complete threat acquisition metadata missing')
        unique=np.flatnonzero(np.any(matrix,axis=0))
        # 같은 옛 반환에 여러 결측 광선이 대응해도 점 자체는 하나다.
        # 절단/표본 추출 없이 전체 위협점을 원래 배경과 같은 시각에 고정한다.
        self.anchor_probe.start(np.asarray(old)[unique],old,ids,reference,source)
        self.clearance_probe.start(np.asarray(old)[unique],old,ids,reference,source)
        self.clearance_probe.bind(self.anchor_probe)
        self.anchor_latch_time=now
        self.anchor_threat_summary=dict(pairs=int(matrix.sum()),unique_points=len(unique),
            source_first_s=source,reference_s=reference,complete=True)

    def _evaluate(self, scan, points, history, now, speed, steering=0., previous_steering=0.,
                 wheelbase=.135, length=.20, width=.15):
        arguments=(scan,points,history,now,speed,steering,previous_steering,wheelbase,length,width)
        result=super().evaluate(*arguments)
        if not self.latched:
            self.anchor_probe.reset();self.anchor_latch_time=None;self.anchor_threat_summary=None
            self.clearance_probe.reset()
            return result
        if (result.get('reason')!='known_obstacle_latched' or
                self.latch_reason!='previous_point_in_missing_sweep'):
            return result
        if (self.anchor_latch_time!=self.latch_time or self.anchor_threat_summary is None):
            # 과거 진단 예시를 전체 상태로 되살리지 않는다.
            return {**result,'anchor_diagnostic':dict(reason='incomplete_threat_set',released=False)}
        first=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
        projected,ids,_=history.project(scan.ranges,scan.angle_min,scan.angle_increment,
            scan.range_min,scan.range_max,first,scan.time_increment,now,mode='both',timing_verified=True)
        if projected.shape != np.asarray(points).shape or not np.allclose(points,projected,rtol=0,atol=1e-8):
            raise ValueError('recovery coordinate mismatch')
        diagnostic=self.anchor_probe.observe(points,ids,history,now,first)
        diagnostic={**diagnostic,'threat_set':self.anchor_threat_summary}
        result={**result,'anchor_diagnostic':{**diagnostic,'released':False}}
        release_field='anchor_diagnostic'
        if not diagnostic['matched']:
            # 옛 위치를 새 유효 반환으로 확인하는 별도 경로. 원래 점 재관측/수명
            # 조건은 그대로이며, 새 경로도 두 스캔 확인 후 현재 입력 재검사가 필요하다.
            scene=motion_scene(scan,history,now,timing_verified=True,rear_half_angle_deg=0.)
            clearance=self.clearance_probe.observe(scene,history,points,ids)
            clearance={**clearance,'threat_set':self.anchor_threat_summary,'released':False}
            result['clearance_diagnostic']=clearance
            if not clearance['matched']:return result
            release_field='clearance_diagnostic'
        # 옛 증거가 다시 보였어도 현재 스캔의 다른 위협/공백을 통과해야 한다.
        trial=copy.deepcopy(self)
        trial.latched=False
        trial.latch_reason=trial.latch_time=trial.latch_evidence=None
        reevaluated=TimedPolicy.evaluate(trial,*arguments)
        result[release_field]['current_recheck_reason']=reevaluated.get('reason')
        if not reevaluated.get('allowed'):
            return result
        # 현재 입력 재검사 통과 때만 후보 상태를 채택한다. 같은 스캔을 추가 확인표로 세지 않는다.
        self.__dict__=trial.__dict__
        self.anchor_probe.reset();self.anchor_latch_time=None;self.anchor_threat_summary=None
        self.clearance_probe.reset()
        diagnostics={key:result[key] for key in ('anchor_diagnostic','clearance_diagnostic') if key in result}
        diagnostics[release_field]={**diagnostics[release_field],'released':True,
            'current_recheck_reason':reevaluated.get('reason')}
        return {**reevaluated,**diagnostics}

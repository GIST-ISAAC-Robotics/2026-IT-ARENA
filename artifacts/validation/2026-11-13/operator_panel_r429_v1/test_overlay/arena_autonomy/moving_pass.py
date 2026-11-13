"""직선 저속 이동차 추월의 선택형 시뮬 후보. 차량 ID/지도/정답 자세를 받지 않는다.

기존 G1/G2 추종을 기본 출력으로 두고, 연속 전진 관측이 있는 앞면에서만 진입한다.
경로/점유/현재 이동영역은 EntryGuide로 다시 검사한다. 상대를 놓쳤다는 이유로
복귀하지 않는다. 후방 현재 반환에 차량 최대 평면 대각선과 측정/시차 여유를 더한
외곽이 자차 뒤에 있고 복귀 띠도 관측될 때만 기존 복귀 곡선을 허용한다.
이 대각선은 20x15 cm 규격의 조건부 외곽 상계이며 물체 신원/미래 운동 보증은 아니다.
첫 범위는 명목 0.30 m/s 상대 한 대와 직선 통로, 자차 명령 최대 0.65 m/s다.
진입 시 상대 속도는 관측 오차를 더한 추정이 0.50 m/s 이하여야 한다.
"""
from dataclasses import dataclass, replace
from collections import deque
import math

import numpy as np

from arena_autonomy.bypass_entry_trial import EntryConfig, EntryGuide, straight_reference
from arena_autonomy.bypass_corridor_trial import abeam_age, wall_frame
from arena_autonomy.bypass_handoff import front_patch
from arena_autonomy.bypass_handoff_corridor_trial import entry_path_conflict
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.known_structure import ACCEPTED_HIDDEN_WIDTH_M, classify, parse_sectors
from arena_autonomy.observation_support import source_valid
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.wall_reference_support import rear_wall_witness


@dataclass(frozen=True)
class MovingPassProfile:
    speed_cap_mps: float
    lead_upper_mps: float
    minimum_advantage_mps: float
    association_speed_mps: float
    measured_entry_envelope: bool = False
    object_motion_bound_mps: float = .5


def moving_pass_profile(name):
    # 초기 저속 결과를 소급 변경하지 않는다. 중속도 명시 선택 시험 범위다.
    if name == 'slow_v1':
        return MovingPassProfile(.65, .50, .15, .75)
    if name == 'medium_v1':
        return MovingPassProfile(.90, .70, .20, 1.)
    if name == 'faster_v1':
        return MovingPassProfile(1.20, 1.00, .20, 1.20, True, 1.20)
    raise ValueError('registered moving pass profile required')


@dataclass(frozen=True)
class MovingEntryConfig(EntryConfig):
    speed_cap_mps: float = .65
    offsets_m: tuple = (0., -.20, .20, -.24, .24, -.27, .27)
    profile: str = 'slow_v1'

    @property
    def measured_speed_envelope(self):
        return moving_pass_profile(self.profile).measured_entry_envelope

    @property
    def maximum_offset_m(self):
        return .35 if self.measured_speed_envelope else .27

    def __post_init__(self):
        ceiling = moving_pass_profile(self.profile).speed_cap_mps
        if (isinstance(self.speed_cap_mps, bool) or not math.isfinite(self.speed_cap_mps)
                or not 0 < self.speed_cap_mps <= ceiling):
            raise ValueError(f'moving pass {self.profile} speed envelope is (0, {ceiling}] m/s')
        # 기존 경로 형상 조건은 유지하고 별도 시험 속도 범위만 정의한다.
        self._validate_geometry()


class MovingTargetTrack:
    """한 후보의 현재 반환 연결. 기억은 대응 탐색용이며 자유 공간/복귀 허가가 아니다."""
    max_gap_s = .4
    association_margin_m = .20
    association_speed_mps = .75
    # 완전히 보이지 않은 부분까지 포함한 규격 직사각형의 최대 평면 길이.
    target_diagonal_m = math.hypot(.20, .15)
    rear_margin_m = .06

    def __init__(self, config, profile='slow_v1'):
        self.config = config
        self.profile_name = profile
        self.profile = moving_pass_profile(profile)
        self.association_speed_mps = self.profile.association_speed_mps
        self.reset()

    def reset(self):
        self.points = None
        self.reference = self.source = self.epoch = None
        self.walls = None
        self.current = None
        self.rear_count = 0
        self.speed_rows = deque(maxlen=5)
        self.speed_source = None
        self.speed_upper = None
        self.speed_lower = None
        self.lost_advantage_scans = 0
        self.detail = dict(observed=False, rear_clear=False, reason='not_started')

    def begin(self, scene, points, walls):
        self.reset()
        self.points = np.asarray(points, dtype=float).copy()
        self.reference, self.source, self.epoch = scene.t_ref_s, scene.source_first_s, scene.source_epoch
        self.walls = []
        for side, wall in walls.items():
            sign = 1. if side == 'left' else -1.
            a = wall['slope']
            b = wall['intercept_m'] + sign*self.config.wall_offset_m*math.hypot(1., a)
            x = np.asarray(wall['observed_x_m'])
            self.walls.append(np.c_[x, a*x+b])
        self.current = self.points.copy()
        self.detail = dict(observed=True, rear_clear=False, reason='front_candidate_started')

    @staticmethod
    def clusters(points, indices):
        if len(points) == 0:
            return []
        # 마스크를 사이에 둔 다른 면을 연결하지 않고 각 관측 조각을 대응시킨다.
        cuts = (np.linalg.norm(np.diff(points, axis=0), axis=1) > .10) | (np.diff(indices) > 2)
        return [p for p in np.split(points, np.flatnonzero(cuts)+1)
                if len(p) >= 3 and np.linalg.norm(np.ptp(p, axis=0)) <= .35]

    def update(self, scene, history):
        try:
            self._update(scene, history)
            self._observe_speed(scene,history)
            return self.detail
        except (ValueError, MotionUnavailable) as error:
            self.reset()
            self.detail = dict(observed=False, rear_clear=False,
                               reason='target_motion_unavailable', error=str(error))
            return self.detail

    def _observe_speed(self,scene,history):
        if self.current is None or not self.detail.get('observed'):
            self.speed_rows.clear();self.speed_source=None
            self.speed_upper=self.speed_lower=None;self.lost_advantage_scans=0
            return
        source=scene.source_first_s
        if source!=self.speed_source:
            self.speed_source=source
            self.speed_rows.append((source,scene.t_ref_s,np.median(self.current,axis=0)))
            if len(self.speed_rows)==5 and source-self.speed_rows[0][0]>=.30:
                points=np.array([transport_points(p[None],history,t,scene.t_ref_s)[0]
                                 for _,t,p in self.speed_rows])
                span=source-self.speed_rows[0][0]
                self.speed_upper=float((points[-1,0]-points[0,0]+2*self.config.measurement_padding_m)/span)
                self.speed_lower=self.speed_upper-4*self.config.measurement_padding_m/span
                # 상대 외곽 하계가 자차 중심보다 앞이면 현재 편이에서 양보할 수 있다.
                # 이미 옆에 있는 차의 보이는 면이 바뀌어 생긴 속도 변화는 쓰지 않는다.
                # 앞 범퍼+여유까지 비었다는 더 강한 조건은 실제 옆 이동/복귀에 적용한다.
                approaching=self.rear_bound(scene)>0.
                lost=self.speed_upper>self.profile.lead_upper_mps and approaching
                self.lost_advantage_scans=self.lost_advantage_scans+1 if lost else 0
            else:
                self.speed_upper=self.speed_lower=None;self.lost_advantage_scans=0
        self.detail.update(observed_speed_upper_mps=self.speed_upper,
            observed_speed_lower_mps=self.speed_lower,
            lost_advantage_scans=self.lost_advantage_scans,
            rear_bound_m=self.rear_bound(scene),ahead_clear=self.ahead_clear(scene))

    def rear_bound(self,scene):
        if self.current is None:return -math.inf
        # 앞면 하나로 차체 후단을 단정하지 않는다. 대응 탐색의 0.75 m/s와
        # 현재 관측 상한 중 큰 값으로 취득 나이를 더한다. 미래 가속 보장은 아니다.
        motion=max(self.association_speed_mps,self.speed_upper or 0.)
        return (float(self.current[:,0].min())-self.target_diagonal_m
                -self.config.measurement_padding_m-motion*max(0.,scene.t_ref_s-self.source))

    def ahead_clear(self,scene):
        return (self.current is not None and self.detail.get('observed',False)
                and self.rear_bound(scene)>self.config.length_m/2+self.rear_margin_m)

    def front_bound(self, points, scene):
        age = max(0., scene.t_ref_s-self.source)
        return (float(np.max(points[:, 0]))+self.target_diagonal_m
                +self.config.measurement_padding_m+self.config.object_motion_bound_mps*age)

    def _update(self, scene, history):
        self.current = None
        if (self.points is None or not source_valid(scene) or scene.source_epoch != self.epoch
                or scene.source_first_s < self.source or scene.t_ref_s < self.reference):
            self.reset()
            return self.detail
        if scene.source_first_s == self.source:
            # 같은 센서 관측을 재사용하되 뒤로 통과한 확인 횟수는 늘리지 않는다.
            if self.detail.get('observed'):
                self.current = transport_points(self.points, history, self.reference, scene.t_ref_s)
                bound = self.front_bound(self.current, scene)
                if bound >= -self.config.length_m/2-self.rear_margin_m:
                    self.rear_count = 0
                self.detail = dict(self.detail, front_bound_m=bound, rear_clear=self.rear_count >= 2,
                                   rear_confirmations=self.rear_count, fresh=False)
            return dict(self.detail, fresh=False)
        elapsed = scene.source_first_s-self.source
        if elapsed > self.max_gap_s:
            self.rear_count = 0
            self.detail = dict(observed=False, rear_clear=False, reason='target_observation_expired')
            return self.detail
        old = transport_points(self.points, history, self.reference, scene.t_ref_s)
        walls = [transport_points(w, history, self.reference, scene.t_ref_s) for w in self.walls]
        rays = [r for r in scene.rays if r.status == 'return']
        points = np.asarray([r.hit_xy_m for r in rays], dtype=float).reshape(-1, 2)
        indices = np.asarray([r.source_index for r in rays])
        keep = np.ones(len(points), dtype=bool)
        for wall in walls:
            delta = wall[-1]-wall[0]
            norm = np.linalg.norm(delta)
            if norm > .1:
                distance = abs(delta[0]*(points[:, 1]-wall[0, 1])-delta[1]*(points[:, 0]-wall[0, 0]))/norm
                keep &= distance > .035
        center = np.median(old, axis=0)
        gate = self.association_margin_m+self.association_speed_mps*elapsed
        keep &= np.linalg.norm(points-center, axis=1) <= gate
        candidates = self.clusters(points[keep], indices[keep])
        ranked = sorted([(float(np.linalg.norm(np.median(p, axis=0)-center)), p) for p in candidates],
                        key=lambda pair: pair[0])
        if not ranked or (len(ranked) > 1 and ranked[1][0]-ranked[0][0] < .06):
            self.rear_count = 0
            self.detail = dict(observed=False, rear_clear=False, reason='target_missing_or_ambiguous',
                               candidates=len(ranked), fresh=True)
            return self.detail
        chosen = ranked[0][1]
        self.points, self.walls, self.reference, self.source = chosen.copy(), walls, scene.t_ref_s, scene.source_first_s
        self.current = chosen.copy()
        # 보이는 점만으로 앞 범퍼를 단정하지 않는다. 어느 반환에서든 규격 대각선까지 더한다.
        front_bound = self.front_bound(chosen, scene)
        behind = front_bound < -self.config.length_m/2-self.rear_margin_m
        self.rear_count = self.rear_count+1 if behind else 0
        self.detail = dict(observed=True, fresh=True, rear_clear=self.rear_count >= 2,
                           rear_confirmations=self.rear_count, front_bound_m=front_bound,
                           observed_bounds_xy=[chosen.min(axis=0).tolist(), chosen.max(axis=0).tolist()],
                           identity_proven=False, reason='current_associated_returns')
        return self.detail



def available_pass_offsets(scene, config, walls, patch, *, fixed_outer_candidate=False, maximum_offset=.27):
    """옆에 섰을 때도 유지할 수 있는 편이를 제안한다. 미관측 부분의 통과 허가는 아니다."""
    half=config.width_m/2
    fixed=.025+config.measurement_padding_m
    # 현재 회전의 취득 시간에 제어 한 주기 여유를 더한다. 조향 추종 오차 1 cm는
    # 첫 두 실험의 .24 m 목표를 조금 넘은 현상을 반영한 시험값이며 실차 상계가 아니다.
    reuse=max(scene.t_ref_s-scene.source_last_s,scene.source_last_s-scene.source_first_s+.02)
    tracking=.01
    offsets=[];report={}
    for side,other,sign in (('left','right',1.),('right','left',-1.)):
        _,normal=wall_frame(patch,config,side,walls[side])
        outer_age=abeam_age(scene,other,reuse)
        inner_age=abeam_age(scene,side,reuse)
        # 명시 후보는 고정 외벽 가설로 편이만 제안한다. EntryGuide의 서로 다른 두 스캔
        # 벽 일치 검사를 통과하기 전에는 주행을 허가하지 않는다. 상대차 여유는 유지한다.
        outer_motion=0. if fixed_outer_candidate else config.object_motion_bound_mps
        low=half+fixed+outer_motion*outer_age+tracking
        high=float(np.min(normal))-half-fixed-config.object_motion_bound_mps*inner_age-tracking
        # 벽에서 자차까지의 거리. 고정된 ±24 cm를 먼저 고르지 않는다.
        center=float(np.clip((low+high)/2,config.wall_offset_m-maximum_offset,config.wall_offset_m))
        offset=sign*(config.wall_offset_m-center)
        feasible=low<=center<=high and abs(offset)>1e-6
        if feasible:offsets.append(offset)
        report[side]=dict(current_observed_width_m=float(np.min(normal)),low_m=low,high_m=high,
                          outer_ray_age_s=outer_age,inner_ray_age_s=inner_age,offset_m=offset,
                          feasible=feasible,tracking_reserve_m=tracking,unknown_filled=False)
    return (0.,*offsets),report


class MovingPassGuide(EntryGuide):
    def __init__(self, config, tracker):
        self.tracker = tracker
        self.raw_scan = None
        self.front_policy='contiguous_three_v0'
        self.sectors = (parse_sectors(config.known_structure_sectors_json)
                        if config.known_structure_policy=='structure_aware_v1' else ())
        super().__init__(config, MovingEntryConfig(speed_cap_mps=tracker.profile.speed_cap_mps,
                         profile=tracker.profile_name, transitions_m=((.9,1.2,1.5) if tracker.profile.measured_entry_envelope
                         else (.6,.9,1.2))), stationary_outer_boundary=True,
                         rejoin_enabled=True, wall_reanchor=True, tracked_wall_fit=True)
        if self.entry.measured_speed_envelope:
            self.padding_provider = self._faster_outer_padding

    def reset(self):
        super().reset()
        self.returning_behind=False

    def evaluate(self,scene,history,speed,previous_steering,dt):
        if self.path is not None or not source_valid(scene):
            result=super().evaluate(scene,history,speed,previous_steering,dt)
            if result.get('entry_phase')=='rejoin' and result.get('rejoin_band'):
                self.returning_behind=result['rejoin_band'].get('return_behind_lead',False)
            return result
        reference=straight_reference(scene,self.config)
        patch,_=front_patch(scene,self.front_policy)
        if reference[0] is None or patch is None:
            self.reset()
            return self._hold('moving_pass_two_walls_and_front_required',reference=reference[1])
        if self.entry.measured_speed_envelope:
            # 긴 외벽을 확인해도 진입 뒤에는 짧은 뒷부분만 남을 수 있다.
            # 그때의 동적 여유를 유지할 수 없는 편이는 처음부터 제안하지 않는다.
            # 후보가 없으면 상위 어댑터가 기존 FOLLOW를 계속 사용한다.
            offsets,width=available_pass_offsets(scene,self.config,reference[1]['walls'],patch,
                maximum_offset=self.entry.maximum_offset_m)
            for row in width.values():
                row.update(fixed_outer_hypothesis=False,requires_distinct_wall_confirmation=True,
                           proposal_policy='dynamic_outer_continuation')
        else:
            offsets,width=available_pass_offsets(scene,self.config,reference[1]['walls'],patch)
        if len(offsets)==1:
            self.reset()
            return self._hold('moving_pass_sustained_side_width_unavailable',pass_width=width)
        original=self.entry
        self.entry=replace(original,offsets_m=offsets)
        self.reference_provider=lambda *args:reference
        try:
            result=super().evaluate(scene,history,speed,previous_steering,dt)
            return {**result,'pass_width':width}
        finally:
            self.entry=original
            self.reference_provider=None

    def _faster_outer_padding(self, scene, config, side, wall, padding_growth_s=0.):
        from arena_autonomy.moving_pass_wall_padding import current_outer_and_rear_padding
        return current_outer_and_rear_padding(scene, config, side, wall, padding_growth_s,
            raw_scan=self.raw_scan, sectors=self.sectors)

    def _outer_wall_fit(self,scene,remembered_wall):
        fit,wall=super()._outer_wall_fit(scene,remembered_wall)
        if (fit is None or fit[2]<=-self.config.wheelbase_m/2 or not self.sectors
                or self.raw_scan is None or len(self.raw_scan.ranges)!=len(scene.rays)):
            return fit,wall
        _,_,rows=classify(self.raw_scan,self.sectors)
        explained={frozenset(row['rays'].tolist()) for row in rows if row['explained']}
        return rear_wall_witness(scene,self.config,self.side,fit,wall,explained)

    def _rejoin_band(self, scene, fit, wall=None):
        band = super()._rejoin_band(scene, fit, wall=wall)
        rear = self.tracker.detail.get('rear_clear', False) and self.tracker.current is not None
        ahead = self.tracker.ahead_clear(scene) and self.tracker.lost_advantage_scans>=3
        return {**band, 'supported': bool(band['supported'] and (rear or ahead)),
                'moving_target_rear_observed': rear,'return_behind_lead':bool(ahead and not rear),
                'target': self.tracker.detail}

    def _confirm_rejoin(self,band,fresh):
        if not band.get('return_behind_lead'):
            return super()._confirm_rejoin(band,fresh)
        # 서로 다른 새 스캔의 확인을 센서 1회 안의 age 증가와 구분한다.
        # 재사용 시 현재 띠가 안 보이면 후보는 내지 않지만 새 관측 이력을
        # 지우지는 않는다. 새 스캔도 실패하면 초기화한다.
        if fresh:
            self.rejoin_confirmations=self.rejoin_confirmations+1 if band['supported'] else 0
        return bool(band['supported'] and self.rejoin_confirmations>=2)

    def _wall_covers_guide(self,fit,wall):
        if super()._wall_covers_guide(fit,wall):
            return True
        # 이미 실제 후방 반환을 확인하고 복귀를 시작한 동안에만 적용한다.
        # 긴 현재 경로의 시작점이 차체 앞부분 안에 있으면 전방 추종 목표를
        # 관측 구간에서 고른다. 시작점을 후륜축까지 외삽하지 않는다.
        # 경로 기준선의 길이 확인이며 실제 이동 영역의 관측/점유 검사는 별도다.
        current_clear=(self.tracker.detail.get('ahead_clear') if self.returning_behind
                       else self.tracker.detail.get('rear_clear'))
        if (self.phase=='rejoin' and current_clear
                and self.tracker.current is not None
                and wall.get('reason')=='current_connected_straight_wall_fit'
                and wall.get('actual_return_count',0)>=12
                and fit[2]<=self.config.length_m/2 and fit[3]>=.65):
            wall['guide_extent']=('current_body_center_and_forward_rejoin_only' if fit[2]<=0.
                                  else 'current_front_body_and_forward_rejoin_only')
            return True
        return False


class MovingPassAdapter:
    """기존 추종 출력과 이동차 경로를 전환. 구조/독립 보호와 점유 검사를 유지한다."""
    max_lead_speed_mps = .50
    minimum_speed_advantage_mps = .15

    def __init__(self, owner):
        self.owner = owner
        self.profile_name = getattr(getattr(owner, 'execution', None), 'moving_pass_profile', 'slow_v1')
        self.profile = moving_pass_profile(self.profile_name)
        if not self.profile.measured_entry_envelope and self.profile.speed_cap_mps > owner.config.max_speed_mps:
            raise ValueError('moving pass profile exceeds entry feedback envelope')
        self.max_lead_speed_mps = self.profile.lead_upper_mps
        self.minimum_speed_advantage_mps = self.profile.minimum_advantage_mps
        self.tracker = MovingTargetTrack(owner.config, self.profile_name)
        self.guide = MovingPassGuide(owner.config, self.tracker)
        self.guide.front_policy=getattr(getattr(owner,'execution',None),'front_patch_policy','contiguous_three_v0')
        self.active = False
        self.admission = {}
        self.handoff_follow = None
        self.handoff_monitor = None
        self.withdrawal_requested = False
        self.withdrawal_speed_cap = self.guide.entry.speed_cap_mps

    def reset(self):
        self.tracker.reset()
        self.guide.reset()
        self.active = False
        self.admission = {}
        self.handoff_follow = None
        self.handoff_monitor = None
        self.withdrawal_requested = False
        self.withdrawal_speed_cap = self.guide.entry.speed_cap_mps

    def _prepare_follow_handoff(self,scene,history,speed,side,previous,dt,raw_scan):
        # 복귀 중 현재 센서로 G1을 먼저 준비한다. 예전 경로의 허가를 옮기지 않는다.
        if self.guide.phase not in ('rejoin','centered'):
            return
        if self.handoff_follow is None:
            from arena_autonomy.local_follow import LocalFollow
            from arena_autonomy.bypass_handoff import BlockingPersistence
            self.handoff_follow=LocalFollow(self.owner.config)
            self.handoff_monitor=BlockingPersistence(self.owner.execution,
                self.owner.config.object_motion_bound_mps,self.owner.config.open_route_speed_mps)
        self.handoff_follow.command(scene,history,speed,side,previous,dt,raw_scan=raw_scan)
        self.handoff_monitor.update(scene,history)

    def _yield_before_return(self,dt):
        # 상대의 속도 이점을 잃어도 곧바로 옆차 쪽으로 꺾지 않는다. 현재 경로의
        # 후보 속도만 완만하게 줄여 앞차가 멀어질 공간을 만든다. 점유 제동은
        # 이 상한과 별개로 더 낮은 목표/정지를 낼 수 있다.
        if self.guide.phase=='entry' and self.tracker.lost_advantage_scans>=3:
            self.withdrawal_requested=True
        if not self.withdrawal_requested:return self.guide.entry
        lower=self.tracker.speed_lower
        desired=self.withdrawal_speed_cap if lower is None else max(.15,min(.40,lower-.05))
        self.withdrawal_speed_cap=min(self.withdrawal_speed_cap,max(desired,self.withdrawal_speed_cap-.5*dt))
        return replace(self.guide.entry,speed_cap_mps=self.withdrawal_speed_cap)

    def eligibility(self, scene, history):
        pace = self.owner.monitor.forward_pace
        if (pace is None or len(pace.rows) < 5 or pace.rate <= 0
                or pace.rows[-1][0] != scene.source_first_s):
            return dict(ready=False, reason='continuous_forward_observation_required')
        span = pace.rows[-1][0]-pace.rows[0][0]
        # 보수 추종율을 역산하면 마지막 구간 감속을 놓친다. 현재 좌표로 옮긴 실제
        # 다섯 관측의 이동량에 같은 거리 오차 예산을 더해 초기 속도 차이를 고른다.
        # 관측 오차에 대한 선택값이지 상대의 미래 가속 상계가 아니다.
        try:
            q = np.asarray([transport_points(p[None], history, ref, scene.t_ref_s)[0]
                            for _, ref, p in pace.rows])
        except (ValueError, MotionUnavailable):
            return dict(ready=False, reason='target_speed_motion_unavailable')
        estimate_upper = (q[-1, 0]-q[0, 0]+pace.bound)/span if span > 0 else math.inf
        tolerance = 1e-12 if self.profile_name != 'slow_v1' else 0.
        ready = (estimate_upper <= self.max_lead_speed_mps + tolerance and
                 self.guide.entry.speed_cap_mps-estimate_upper >= self.minimum_speed_advantage_mps - tolerance)
        return dict(ready=ready, reason='slow_forward_candidate' if ready else 'speed_advantage_insufficient',
                    estimated_lead_upper_mps=estimate_upper, source_span_s=span,
                    pass_speed_cap_mps=self.guide.entry.speed_cap_mps,
                    scope=('initial_straight_slow_target_only' if self.profile_name == 'slow_v1'
                           else ('initial_straight_medium_target_only' if self.profile_name == 'medium_v1' else 'initial_straight_faster_target_only')), future_speed_guaranteed=False)

    def entry_advantage_sufficient(self, chosen_speed, estimated_lead_upper):
        # 기존 저속 후보의 >0 조건은 재현용으로 유지한다. 새 중속은 실제 곡률/점유로
        # 제한된 후보가 0.20m/s 이점을 갖춰야 한다. 상한 숫자만 비교하지 않는다.
        required = self.minimum_speed_advantage_mps if self.profile_name != 'slow_v1' else 0.
        return (math.isfinite(chosen_speed) and math.isfinite(estimated_lead_upper)
                and chosen_speed > estimated_lead_upper
                and chosen_speed - estimated_lead_upper >= required - 1e-12)

    def command(self, scene, history, speed, side, previous, dt, *, raw_scan=None):
        owner = self.owner
        self.guide.raw_scan = raw_scan
        if (raw_scan is None or not source_valid(scene) or side not in ('left','right')
                or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
                           for v in (speed,previous,dt))
                or abs(previous)>owner.config.max_steering_rad or not 0<dt<=.2):
            self.reset()
            return owner._base_command(scene,history,speed,side,previous,dt,raw_scan=raw_scan)
        if not self.active:
            normal = owner._base_command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
            gate = self.eligibility(scene, history)
            detail = {**normal[2], 'moving_pass': gate}
            if not gate['ready'] or owner.mode != 'FOLLOW' or abs(previous) > .08:
                self.guide.reset()
                return normal[0], normal[1], detail
            patch, _ = front_patch(scene, owner.execution.front_patch_policy)
            if patch is None:
                self.guide.reset()
                return normal[0], normal[1], detail
            proposal = self.guide.evaluate(scene, history, speed, previous, dt)
            chosen = proposal.get('selected')
            detail['moving_pass']['entry'] = proposal
            if not chosen or not chosen['offset_m']:
                return normal[0], normal[1], detail
            conflict = entry_path_conflict(dict(path_points=self.guide.path), scene, owner.config)
            gate['path_conflict'] = conflict
            if not self.entry_advantage_sufficient(chosen['speed_mps'], gate['estimated_lead_upper_mps']) or conflict['rejected']:
                self.guide.reset()
                gate['reason'] = 'entry_speed_or_path_conflict'
                return normal[0], normal[1], detail
            selected_side = 'left' if chosen['offset_m'] > 0 else 'right'
            threat = normal[2].get('normal', {}).get('selector_sides', {}).get(selected_side)
            if threat is None or threat.get('present') or threat.get('seen'):
                self.guide.reset()
                detail['moving_pass']['reason'] = 'current_side_threat_or_unavailable'
                return normal[0], normal[1], detail
            self.tracker.begin(scene, patch, proposal['reference']['walls'])
            self.active = True
            self.admission = {k:v for k,v in gate.items() if k != 'entry'}
            return self._output(chosen, proposal, 'moving_pass_entry')
        target = self.tracker.update(scene, history)
        if not source_valid(scene) or scene.source_epoch != self.tracker.epoch:
            self.reset()
            owner.normal.reset()
            return 0., previous, dict(reason='moving_pass_source_reset', behavior='HOLD',
                                     mode='g1_g2_handoff_trial', handoff_state='MOVING_PASS', simulation_trial_only=True,
                                     output_requires_independent_guard=True)
        original=self.guide.entry
        self.guide.entry=self._yield_before_return(dt)
        try:
            proposal = self.guide.evaluate(scene, history, speed, previous, dt)
        finally:
            self.guide.entry=original
        chosen = proposal.get('selected')
        self._prepare_follow_handoff(scene,history,speed,side,previous,dt,raw_scan)
        if proposal.get('entry_phase') == 'centered':
            behind=self.guide.returning_behind
            prepared,monitor=self.handoff_follow,self.handoff_monitor
            owner.reset()
            # owner.reset()은 모드/우회 상태를 지운다. 같은 입력으로 따로 준비한
            # 경계·측방/전방 관측은 버리지 않는다. 실제 정지는 G1이 그대로 판단한다.
            if prepared is not None:
                owner.normal,owner.monitor=prepared,monitor
            normal = owner._base_command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
            return normal[0], normal[1], {**normal[2], 'moving_pass':dict(
                reason='aborted_to_observed_follow' if behind else 'rejoined', completed=not behind)}
        if chosen is None:
            return 0., previous, dict(reason='moving_pass_current_path_unavailable', behavior='HOLD',
                                     mode='g1_g2_handoff_trial', handoff_state='MOVING_PASS', simulation_trial_only=True,
                                     moving_pass=dict(target=target, entry=proposal),
                                     output_requires_independent_guard=True)
        why=('moving_pass_return_behind' if self.guide.returning_behind else
             'moving_pass_yield_for_return' if self.withdrawal_requested else 'moving_pass_'+self.guide.phase)
        return self._output(chosen, proposal, why)

    def _output(self, chosen, proposal, reason):
        return chosen['speed_mps'], chosen['steering_rad'], dict(
            reason=reason, behavior='MOVING_PASS', mode='g1_g2_handoff_trial',
            handoff_state='MOVING_PASS', simulation_trial_only=True,
            output_requires_independent_guard=True,
            moving_pass=dict(admission=self.admission, phase=self.guide.phase,
                             withdrawal_requested=self.withdrawal_requested,
                             withdrawal_speed_cap_mps=self.withdrawal_speed_cap,
                             target=self.tracker.detail, entry=proposal))

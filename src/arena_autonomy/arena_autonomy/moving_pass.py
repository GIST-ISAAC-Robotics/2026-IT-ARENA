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


@dataclass(frozen=True)
class MovingEntryConfig(EntryConfig):
    speed_cap_mps: float = .65
    offsets_m: tuple = (0., -.20, .20, -.24, .24, -.27, .27)

    def __post_init__(self):
        if (isinstance(self.speed_cap_mps, bool) or not math.isfinite(self.speed_cap_mps)
                or not 0 < self.speed_cap_mps <= .65):
            raise ValueError('moving pass initial speed envelope is (0, .65] m/s')
        # 기존 경로 형상 조건은 유지하고 별도 시험 속도 범위만 정의한다.
        EntryConfig(self.offsets_m, self.transitions_m, min(.35, self.speed_cap_mps),
                    self.path_step_m, self.prefer_earlier_transition)


class MovingTargetTrack:
    """한 후보의 현재 반환 연결. 기억은 대응 탐색용이며 자유 공간/복귀 허가가 아니다."""
    max_gap_s = .4
    association_margin_m = .20
    association_speed_mps = .75
    # 완전히 보이지 않은 부분까지 포함한 규격 직사각형의 최대 평면 길이.
    target_diagonal_m = math.hypot(.20, .15)
    rear_margin_m = .06

    def __init__(self, config):
        self.config = config
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
                lost=self.speed_upper>.50 and approaching
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


def rear_wall_witness(scene, config, side, fit, wall, explained_sectors):
    """지지대 양옆에서 실제로 보인 면이 일치할 때만 후륜축의 경로 기준을 연장한다.

    뒤쪽 짧은 면은 작은 차량 옆면일 수도 있다. 긴 벽에 붙이거나 고정 점유로
    분류하지 않는다. 원래의 고정 여유 적용 광선 목록과 가림 상태도 보존한다.
    """
    if fit is None or wall.get('reason') != 'current_connected_straight_wall_fit':
        return fit, wall
    a,b,lower,upper=fit
    if lower <= -config.wheelbase_m/2 or upper < .30 or upper-lower < .65:
        return fit, wall
    ids=wall['source_ray_indices']
    rays={r.source_index:r for r in scene.rays}
    if not ids or ids[0] not in rays or ids[-1] not in rays:
        return fit, wall
    # 광선 획득 순서는 좌/우에서 반대다. 차체 뒤쪽 끝에서 바깥 방향으로 검사한다.
    edge,step=min(((ids[0],-1),(ids[-1],1)), key=lambda e:rays[e[0]].hit_xy_m[0])
    sign=1 if side=='left' else -1
    norm=math.hypot(1.,a)
    actual_b=b+sign*config.wall_offset_m*norm
    shift=sign*config.wall_offset_m*a/norm
    i=edge+step
    gap=[]
    while i in rays and rays[i].status=='masked':
        gap.append(i);i+=step
    if not gap or frozenset(gap) not in explained_sectors or i not in rays or rays[i].status!='return':
        return fit,wall
    chord=math.dist(rays[edge].hit_xy_m,rays[i].hit_xy_m)
    if chord>ACCEPTED_HIDDEN_WIDTH_M:
        return fit,wall
    points=[];last=rays[edge].hit_xy_m
    while i in rays and rays[i].status=='return':
        p=rays[i].hit_xy_m
        if (not -.40<=p[0]<last[0] or math.dist(last,p)>.05
                or abs(p[1]-a*p[0]-actual_b)/norm>.015):
            break
        points.append(p);last=p;i+=step
    if len(points)<12 or points[0][0]-points[-1][0]<.20:
        return fit,wall
    x,y=np.asarray(points).T
    observed_slope=np.dot(x-x.mean(),y-y.mean())/np.dot(x-x.mean(),x-x.mean())
    new_lower=float(min(x)+shift)
    if new_lower > -config.wheelbase_m/2 or abs(math.atan(observed_slope)-math.atan(a))>.08:
        return fit,wall
    proof=dict(principal_x_m=[lower,upper],observed_rear_x_m=[float(min(x)),float(max(x))],
               rear_return_count=len(points),mask_indices=sorted(gap),shadow_chord_m=chord,
               short_piece_static=False,unknown_filled=False)
    return (a,b,new_lower,upper),{**wall,'observed_x_m':[new_lower,upper],'rear_geometry_witness':proof}


def available_pass_offsets(scene, config, walls, patch):
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
        low=half+fixed+config.object_motion_bound_mps*outer_age+tracking
        high=float(np.min(normal))-half-fixed-config.object_motion_bound_mps*inner_age-tracking
        # 벽에서 자차까지의 거리. 고정된 ±24 cm를 먼저 고르지 않는다.
        center=float(np.clip((low+high)/2,config.wall_offset_m-.27,config.wall_offset_m))
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
        super().__init__(config, MovingEntryConfig(), stationary_outer_boundary=True,
                         rejoin_enabled=True, wall_reanchor=True, tracked_wall_fit=True)

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

    def _outer_wall_fit(self,scene,remembered_wall):
        fit,wall=super()._outer_wall_fit(scene,remembered_wall)
        if (fit is None or fit[2]<=-self.config.wheelbase_m/2 or not self.sectors
                or self.raw_scan is None or len(self.raw_scan.ranges)!=len(scene.rays)):
            return fit,wall
        _,_,rows=classify(self.raw_scan,self.sectors)
        explained={frozenset(row['rays'].tolist()) for row in rows if row['explained']}
        return rear_wall_witness(scene,self.config,self.side,fit,wall,explained)

    def _rejoin_band(self, scene, fit):
        band = super()._rejoin_band(scene, fit)
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
        # 차체 중심과 앞쪽의 긴 벽이 지금 보이면 Pure Pursuit 목표를 그 안에서
        # 고를 수 있다. 관측 시작점을 후륜축까지 연장하지 않고 실제 구간만 자른다.
        current_clear=(self.tracker.detail.get('ahead_clear') if self.returning_behind
                       else self.tracker.detail.get('rear_clear'))
        if (self.phase=='rejoin' and current_clear
                and self.tracker.current is not None
                and wall.get('reason')=='current_connected_straight_wall_fit'
                and wall.get('actual_return_count',0)>=12
                and fit[2]<=0. and fit[3]>=.65):
            wall['guide_extent']='current_body_center_and_forward_rejoin_only'
            return True
        return False


class MovingPassAdapter:
    """기존 추종 출력과 이동차 경로를 전환. 구조/독립 보호와 점유 검사를 유지한다."""
    max_lead_speed_mps = .50
    minimum_speed_advantage_mps = .15

    def __init__(self, owner):
        self.owner = owner
        self.tracker = MovingTargetTrack(owner.config)
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
        ready = (estimate_upper <= self.max_lead_speed_mps and
                 self.guide.entry.speed_cap_mps-estimate_upper >= self.minimum_speed_advantage_mps)
        return dict(ready=ready, reason='slow_forward_candidate' if ready else 'speed_advantage_insufficient',
                    estimated_lead_upper_mps=estimate_upper, source_span_s=span,
                    pass_speed_cap_mps=self.guide.entry.speed_cap_mps,
                    scope='initial_straight_slow_target_only', future_speed_guaranteed=False)

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
            if chosen['speed_mps'] <= gate['estimated_lead_upper_mps'] or conflict['rejected']:
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

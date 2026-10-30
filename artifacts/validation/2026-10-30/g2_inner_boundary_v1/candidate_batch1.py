"""오프라인 G2 중앙 통로 후보. 운영/기본값에 연결하지 않는다.

세 요소를 독립 스위치로 둔다.
- surface: 같은 표면의 광선 산포를 표면 기준으로 정렬한다. 고정 55 mm(거리 치우침 30 mm 포함)는
  표면 기준으로 그대로 남긴다. 각 광선의 차감은 30 mm 이하이고 벽/표면 밖 반환은 건드리지 않는다.
- plan: 순차 스캔 시각표로 옆면 광선 나이가 짧은 쪽과 통로 중앙에 가까운 등록 편이를 고른다.
- onset: 안쪽 경계가 외벽 기준 최근 창에서 다가오지 않았을 때만 0.5 m/s*age 대신
  측정 저속 상한*age + 시작 가속 상한*age^2/2를 쓴다. 시작 가속 3 m/s^2는 가정이다.

추종, 관측 지지, 독립 보호, STOP, 차량/센서 모델은 건드리지 않는다.
물체 ID, 정지, 자유 공간을 증명하지 않는다.
"""
from dataclasses import asdict, dataclass, replace
import math

import numpy as np

import arena_autonomy.bypass_entry_trial as bt
from arena_autonomy.bypass_entry_trial import EntryConfig, EntryGuide, straight_reference
from arena_autonomy.observation_support import source_valid


@dataclass(frozen=True)
class CorridorPolicy:
    surface: bool = True
    plan: bool = True
    onset: bool = False
    noise_budget_m: float = .03       # 기존 C1 ±30 mm 값. 새 측정 주장이 아님
    face_minimum_returns: int = 8
    face_bin_m: float = .05
    face_minimum_bins: int = 3
    quantile_rank: int = 2            # 낮은 쪽 3번째. 단일 이상점 회피
    along_min_m: float = -.45
    along_max_m: float = 1.60
    normal_min_m: float = .06
    normal_max_m: float = .78
    window_s: float = .40             # onset 근거 창, 10 Hz 서로 다른 5스캔
    minimum_scans: int = 5
    onset_acceleration_mps2: float = 3.  # 가정, 실측 아님
    scan_reuse_s: float = .08         # 50 Hz 제어의 같은 10 Hz 스캔 재사용 상한(시뮬레이션 일정)
    side_age_margin_s: float = .02    # 이보다 작은 나이 차이는 기존 순위에 맡긴다


def wall_frame(points, config, side, wall):
    """외벽에서 안쪽(+)으로 잰 법선 거리와 벽 방향 좌표. 같은 스캔 안의 상대량이다."""
    sign=1 if side=='left' else -1
    slope,intercept=wall['slope'],wall['intercept_m']
    norm=math.sqrt(1+slope*slope)
    physical=intercept+sign*config.wall_offset_m*norm
    return (points[:,0]+slope*points[:,1])/norm,sign*(slope*points[:,0]+physical-points[:,1])/norm


def inner_surface(along, normal, candidates, policy):
    """외벽에 가장 가까운 비벽 반환의 위치. 벽과 나란한 면이 충분할 때만 산포 중앙값을 쓴다."""
    window=candidates&(along>=policy.along_min_m)&(along<=policy.along_max_m)&(
        normal>=policy.normal_min_m)&(normal<=policy.normal_max_m)
    if window.sum()<=policy.quantile_rank:
        return None
    low=float(np.partition(normal[window],policy.quantile_rank)[policy.quantile_rank])
    band=window&(normal<=low+2*policy.noise_budget_m)
    bins=np.floor(along[band]/policy.face_bin_m).astype(int)
    labels=np.unique(bins)
    if band.sum()>=policy.face_minimum_returns and len(labels)>=policy.face_minimum_bins:
        # 벽에 수직인 뒷면은 한 벽 방향 구간에 몰린다. 구간별 대표값으로 면 표본을 동등하게 센다.
        value=float(np.median([np.median(normal[band][bins==label]) for label in labels]))
        return dict(normal_m=value,method='parallel_face_median',low_order_m=low,returns=int(band.sum()),bins=len(labels))
    return dict(normal_m=low,method='low_order',low_order_m=low,returns=int(window.sum()),bins=len(labels))


class BoundaryMonitor:
    """외벽 기준 안쪽 경계의 스캔별 위치. 새 스캔에서만 누적한다."""
    def __init__(self,policy):
        self.policy=policy;self.reset()

    def reset(self):
        self.rows=[];self.key=None;self.state=dict(stable=False,reason='no_observation')

    def update(self,scene,surface):
        key=(scene.source_epoch,scene.source_first_s)
        if key==self.key:return self.state
        if self.key is not None and (key[0]!=self.key[0] or key[1]<self.key[1] or key[1]-self.key[1]>.25):
            self.rows=[]
        self.key=key
        p=self.policy
        if surface is None:
            self.rows=[]
            self.state=dict(stable=False,reason='inner_boundary_not_observed')
            return self.state
        current=surface['normal_m']
        self.rows.append((scene.source_first_s,current))
        latest=self.rows[-1][0]
        while len(self.rows)>p.minimum_scans and latest-self.rows[1][0]>=p.window_s-1e-9:
            self.rows.pop(0)
        span=latest-self.rows[0][0]
        approach=max(v for _,v in self.rows)-current
        stable=len(self.rows)>=p.minimum_scans and span>=p.window_s-1e-9 and approach<=p.noise_budget_m
        self.state=dict(stable=stable,inner_normal_m=current,approach_m=approach,span_s=span,scans=len(self.rows),
            rate_bound_mps=(p.noise_budget_m/span if span>0 else None),
            reason='low_approach_inner_boundary_candidate' if stable else 'inner_boundary_window_or_approach',
            object_identity_or_stationarity_proven=False,free_space_assumed=False)
        return self.state


def abeam_age(scene, side, reuse_s):
    """통과 쪽 옆면 방위 광선의 최악 나이. 표본 시각표만 쓰며 물체를 가정하지 않는다."""
    goal=-1. if side=='left' else 1.   # 왼쪽 통과면 표적은 오른쪽 옆면
    rays=[ray for ray in scene.rays if ray.status!='masked']
    best=max(rays,key=lambda ray:goal*ray.direction_xy[1])
    return float(scene.source_last_s-best.acquired_at_s+reuse_s)


class CorridorGuide(EntryGuide):
    def __init__(self,config=None,entry=None,*,corridor_policy=None,**kwargs):
        self.corridor_policy=corridor_policy or CorridorPolicy()
        self.registered_entry=entry
        super().__init__(config,entry,**kwargs)
        self.registered_entry=self.entry

    def reset(self):
        super().reset()
        self.monitors=dict(left=BoundaryMonitor(self.corridor_policy),right=BoundaryMonitor(self.corridor_policy))
        self.corridor_use={};self.plan_detail=None

    def _padding(self,original,scene,config,side,wall,growth_s=0.):
        padding,evidence=original(scene,config,side,wall,growth_s)
        if growth_s!=0.:
            raise ValueError('offline candidate has no growth comparison')
        p=self.corridor_policy
        padding=np.asarray(padding,dtype=float).copy()
        before=padding.copy()
        points=np.asarray(scene.occupied_endpoints_xy_m,dtype=float).reshape(-1,2)
        rays=[ray for ray in scene.rays if ray.status=='return']
        fixed=.025+config.measurement_padding_m
        static=padding<=fixed+1e-12
        along,normal=wall_frame(points,config,side,wall)
        surface=inner_surface(along,normal,~static,p)
        state=self.monitors[side].update(scene,surface)
        use=dict(side=side,surface=surface,state=state,wall_rays_aligned=0,inner_rays_aligned=0,onset_rays=0)
        supported=np.zeros(len(points),dtype=bool)
        if surface is not None:
            supported=(~static)&(normal>=surface['normal_m']-p.noise_budget_m)&(
                along>=p.along_min_m)&(along<=p.along_max_m)&(normal<=p.normal_max_m)
        if p.onset and state.get('stable') and supported.any():
            ages=np.asarray([scene.t_ref_s-ray.acquired_at_s for ray in rays],dtype=float)
            reduced=fixed+state['rate_bound_mps']*ages+.5*p.onset_acceleration_mps2*ages*ages
            padding[supported]=np.minimum(padding[supported],reduced[supported])
            use['onset_rays']=int(supported.sum())
        if p.surface:
            wall_snap=np.clip(normal,0.,p.noise_budget_m)*static
            padding-=wall_snap
            use['wall_rays_aligned']=int(np.count_nonzero(wall_snap))
            if surface is not None:
                inner_snap=np.clip(surface['normal_m']-normal,0.,p.noise_budget_m)*supported
                padding-=inner_snap
                use['inner_rays_aligned']=int(np.count_nonzero(inner_snap))
        use['maximum_inner_padding_m']=float(np.max(padding[supported])) if supported.any() else None
        use['padding_never_increased']=bool(np.all(padding<=before+1e-12))
        self.corridor_use[side]=use
        return padding,dict(evidence,corridor=use)

    def _plan(self,scene):
        """첫 진입의 쪽/편이 비교. 등록 편이만 쓰고 비교 결과는 허가가 아니다."""
        p=self.corridor_policy
        reference,evidence=straight_reference(scene,self.config)
        if reference is None or not source_valid(scene):
            return None
        points=np.asarray(scene.occupied_endpoints_xy_m,dtype=float).reshape(-1,2)
        rows={}
        for side in ('left','right'):
            wall=evidence['walls'][side]
            along,normal=wall_frame(points,self.config,side,wall)
            surface=inner_surface(along,normal,np.ones(len(points),dtype=bool),p)
            if surface is None:
                return None
            age=abeam_age(scene,side,p.scan_reuse_s)
            motion=(.5*p.onset_acceleration_mps2*age*age+p.noise_budget_m/p.window_s*age if p.onset
                    else self.config.object_motion_bound_mps*age)
            fixed=.025+self.config.measurement_padding_m
            low=self.config.width_m/2+fixed
            high=surface['normal_m']-self.config.width_m/2-fixed-motion
            center=(low+high)/2
            sign=1. if side=='left' else -1.
            want=sign*(self.config.wall_offset_m-center)
            options=[o for o in self.registered_entry.offsets_m if o*sign>0]
            rows[side]=dict(side=side,age_s=age,inner_normal_m=surface['normal_m'],low_m=low,high_m=high,
                width_m=high-low,want_offset_m=want,offset_m=min(options,key=lambda o:abs(o-want)) if options else None,
                surface=surface['method'])
        ages=[rows['left']['age_s'],rows['right']['age_s']]
        chosen=None
        if abs(ages[0]-ages[1])>=p.side_age_margin_s:
            chosen='left' if ages[0]<ages[1] else 'right'
        elif abs(rows['left']['width_m']-rows['right']['width_m'])>.005:
            chosen=max(rows,key=lambda side:rows[side]['width_m'])
        return dict(rows=rows,chosen=chosen,side_comparison_is_permission=False)

    def evaluate(self,scene,history,speed,previous,dt):
        self.corridor_use={}
        original=bt.current_outer_wall_padding
        bt.current_outer_wall_padding=lambda *a,**k:self._padding(original,*a,**k)
        try:
            if self.path is None and self.corridor_policy.plan:
                result=self._first_selection(scene,history,speed,previous,dt)
            else:
                result=super().evaluate(scene,history,speed,previous,dt)
        finally:
            bt.current_outer_wall_padding=original
        return {**result,'corridor':dict(self.corridor_use),'corridor_plan':self.plan_detail}

    def _first_selection(self,scene,history,speed,previous,dt):
        plan=self._plan(scene)
        self.plan_detail=plan
        if plan is None or plan['chosen'] is None or plan['rows'][plan['chosen']]['offset_m'] is None:
            return super().evaluate(scene,history,speed,previous,dt)
        offset=plan['rows'][plan['chosen']]['offset_m']
        self.entry=replace(self.registered_entry,offsets_m=(0.,offset))
        try:
            result=super().evaluate(scene,history,speed,previous,dt)
        finally:
            self.entry=self.registered_entry
        if self.path is None or not self.corridor_policy.onset:
            return result
        if self.corridor_use.get(self.side,{}).get('state',{}).get('stable'):
            return result
        # 저속 근거 없이 경로를 확정하지 않는다. 다음 호출에서 같은 근거를 다시 본다.
        self.path=self.wall_line=self.reference_s=None
        self.offset=self.transition=self.side=None
        self.wall_confirmations=0
        self.intent.sign=None
        return self._hold('entry_inner_boundary_evidence_wait',reference=result.get('reference'),
            guide_mode='new_observed_entry')


def policy_dict(policy):
    return asdict(policy)

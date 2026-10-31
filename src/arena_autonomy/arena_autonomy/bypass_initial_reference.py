"""명시 G2 초기 기준 보완. 현재 연결 반환과 기존 준비 완료를 함께 요구한다."""
import numpy as np
from arena_autonomy.bypass_entry_trial import straight_reference
from arena_autonomy.bypass_wall_fit import tracked_straight_wall
from arena_autonomy.observation_support import source_valid
from arena_autonomy.scene_motion import transport_points

def line_of(fit):
    a,b,lo,hi=fit
    return np.array([[lo,a*lo+b],[hi,a*hi+b]])


class CurrentPairReference:
    def __init__(self):self.clear()

    def clear(self):
        self.anchors={};self.key=None;self.last_ref=None

    def resolve(self,scene,config,history):
        original,detail=straight_reference(scene,config)
        now,source=scene.t_ref_s,scene.source_first_s
        if not source_valid(scene) or source is None or not 0<=now-source<.30:
            self.clear();return original,detail
        key=(scene.source_frame_id,scene.source_epoch,source)
        if self.key is not None and (key[:2]!=self.key[:2] or source<self.key[2]
                or now<self.last_ref or source-self.key[2]>.25):self.clear()
        fresh=key!=self.key;self.key=key;self.last_ref=now
        if original is not None:
            if fresh:
                for side,wall in detail['walls'].items():
                    fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
                    self.anchors[side]=dict(line=line_of(fit),time=now)
            return original,detail
        fits=[];walls={}
        for side in ('left','right'):
            old=self.anchors.get(side);fit=None;evidence=dict(reason='no_recent_seed')
            if old is not None and 0<=now-old['time']<=1.:
                try:
                    line=transport_points(old['line'],history,old['time'],now)
                    fit,evidence=tracked_straight_wall(scene,config,side,line)
                except ValueError:evidence=dict(reason='seed_transport_unavailable')
            fits.append(fit);walls[side]=evidence
            if fit is not None and fresh:self.anchors[side]=dict(line=line_of(fit),time=now)
        evidence=dict(original=detail,walls=walls,all_returns_remain_occupied=True,
            remembered_line_is_current_observation=False)
        if any(f is None for f in fits):return None,dict(reason='current_pair_unavailable',**evidence)
        a,b=fits;lower,upper=max(a[2],b[2]),min(a[3],b[3])
        if lower>-config.wheelbase_m/2 or upper-max(lower,0.)<.25:
            return None,dict(reason='reference_does_not_cover_entry',**evidence)
        disagreement=max(abs((a[0]-b[0])*x+a[1]-b[1]) for x in (lower,upper))
        if disagreement>config.boundary_agreement_m:
            return None,dict(reason='observed_wall_reference_disagrees',**evidence)
        return ((a[0]+b[0])/2,(a[1]+b[1])/2,lower,upper),dict(
            reason='current_tracked_pair_candidate',disagreement_m=float(disagreement),
            wall_offset_assumption_m=config.wall_offset_m,**evidence)


class DirectionalPairReference(CurrentPairReference):
    def resolve(self, scene, config, history):
        reference, detail = super().resolve(scene, config, history)
        if (reference is not None or detail.get('reason') != 'reference_does_not_cover_entry'
                or detail.get('all_returns_remain_occupied') is not True):
            return reference, detail
        walls = detail['walls']
        a, b = (walls[side] for side in ('left', 'right'))
        lower = max(a['observed_x_m'][0], b['observed_x_m'][0])
        upper = min(a['observed_x_m'][1], b['observed_x_m'][1])
        if upper-max(lower, 0.) < .25:
            return reference, detail
        rear = -config.wheelbase_m/2
        covering = [side for side, wall in walls.items()
                    if wall['observed_x_m'][0] <= rear and wall['observed_x_m'][1]-rear >= .30]
        if not covering:
            return reference, detail
        disagreement = max(abs((a['slope']-b['slope'])*x+a['intercept_m']-b['intercept_m'])
                           for x in (lower, upper))
        if disagreement > config.boundary_agreement_m:
            return None, dict(detail, reason='observed_wall_reference_disagrees',
                              disagreement_m=float(disagreement))
        # 이 공통 구간을 뒤로 늘리지 않는다. 실행 경로는 선택한 외벽의 실제 구간을 쓴다.
        result = ((a['slope']+b['slope'])/2, (a['intercept_m']+b['intercept_m'])/2, lower, upper)
        return result, dict(detail, reason='current_pair_directional_coverage_candidate',
            disagreement_m=float(disagreement), entry_covering_sides=covering,
            common_observed_x_m=[lower, upper],
            path_requires_selected_wall_coverage=True, unknown_filled=False,
            wall_offset_assumption_m=config.wall_offset_m)


class ReadyInitialReference:
    """한 판단 호출 안에서만 공유하는 기준 공급자. 과거 선은 탐색 위치다."""
    def __init__(self, *, directional_coverage=False):
        if type(directional_coverage) is not bool:
            raise ValueError("explicit directional coverage selection required")
        self.memory=DirectionalPairReference() if directional_coverage else CurrentPairReference()
        self.clear()

    def clear(self):
        self.memory.clear();self.scene=None;self.config=None
        self.original=self.proposed=None

    def update(self,scene,config,history):
        self.scene,self.config=scene,config
        self.proposed=self.memory.resolve(scene,config,history)
        self.original=straight_reference(scene,config)

    def reference(self,scene,config,monitor):
        # 다른 시각/설정의 호출에는 캐시를 넘기지 않는다.
        if scene is not self.scene or config is not self.config:
            return straight_reference(scene,config)
        if self.original[0] is not None or self.proposed[0] is None:
            return self.original
        result=monitor.result
        ready=(result.get('observed') is True and result.get('ready') is True
               and not monitor.motion_evidence())
        return self.proposed if ready else self.original

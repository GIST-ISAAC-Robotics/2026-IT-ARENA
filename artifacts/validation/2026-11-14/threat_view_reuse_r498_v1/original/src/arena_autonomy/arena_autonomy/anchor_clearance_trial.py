"""선택형 후보: 옛 점 주변의 현재 실제 반환 지지. 물체 추적/자유 공간 증명은 아니다.

이동 물체의 옛 좌표를 정적인 재관측만으로 해제할 수 없는 경우를 비교한다.
현재의 알려진 점유와 unknown/masked는 그대로 두며 명령을 생성하지 않는다.
"""
import math
import numpy as np
from arena_autonomy.observation_support import depth_support,source_valid
from arena_autonomy.partial_surface_alignment import surface_translation
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial


def fresh_clearance(retained, background, original_reference, scene, history, current, ids):
    result=dict(accepted=False,unknown_filled=False,scope='old_location_clearance_not_object_identity',
        identity_proven=False,residual_hidden_object_risk=True,
        coordinate_radius_m=.03,return_margin_m=.03,motion_bound_mps=2.,maximum_history_s=1.)
    def finish(reason,**extra):return {**result,**extra,'reason':reason}
    retained=np.asarray(retained,dtype=float)
    background=np.asarray(background,dtype=float)
    current=np.asarray(current,dtype=float)
    ids=np.asarray(ids)
    if (retained.ndim!=2 or retained.shape[1]!=2 or not 1<=len(retained)<=1024
            or not np.all(np.isfinite(retained)) or not math.isfinite(original_reference)
            or not source_valid(scene) or not 0<=scene.t_ref_s-original_reference<=1.):
        return finish('invalid_or_expired')
    if (background.ndim!=2 or background.shape[1]!=2 or not 50<=len(background)<=1024
            or current.ndim!=2 or current.shape[1]!=2 or not 50<=len(current)<=1024
            or not np.all(np.isfinite(background)) or not np.all(np.isfinite(current))
            or ids.shape!=(len(current),) or not np.issubdtype(ids.dtype,np.integer)
            or np.any(np.diff(ids)<=0) or np.any(ids<0) or np.any(ids>=len(scene.rays))):
        return finish('invalid_point_metadata')
    if (any(scene.rays[i].status!='return' for i in ids)
            or not np.allclose(current,[scene.rays[i].hit_xy_m for i in ids],rtol=0,atol=1e-8)):
        return finish('scene_point_mismatch')
    carried=transport_points(background,history,original_reference,scene.t_ref_s)
    old=transport_points(retained,history,original_reference,scene.t_ref_s)
    fit=dict(surface_translation(carried,current,ids))
    # 비용은 호출 외부에서 계측한다. 판단 진단은 결정론적 필드만 반환한다.
    fit.pop('processing_ms',None)
    if not fit['accepted']:return finish('surface_rejected',fit=fit)
    old=old+np.asarray(fit['translation_m'])
    # 원래 점과3cm 주변의1cm 격자/외곽 표본. 연속 영역 인증으로 확대하지 않는다.
    grid=np.array([(x,y) for x in np.arange(-3,4)*.01 for y in np.arange(-3,4)*.01
        if x*x+y*y<=.03**2+1e-12])
    theta=np.arange(24)*math.tau/24
    offsets=np.vstack((grid,.03*np.column_stack((np.cos(theta),np.sin(theta)))))
    padding=np.array([.03+2*(scene.t_ref_s-r.acquired_at_s) for r in scene.rays])
    valid=np.array([r.status=='return' for r in scene.rays])
    hits=np.array([r.hit_xy_m for r in scene.rays if r.status=='return']).reshape(-1,2)
    proofs=[]
    for point in old:
        queries=point+offsets
        support=depth_support(scene,queries,padding)
        occupied=bool(len(hits) and np.any(np.linalg.norm(hits-point,axis=1)<=padding[valid]+.03))
        proofs.append(dict(supported_queries=int(support.sum()),queries=len(queries),
            occupied=occupied,clear=bool(np.all(support) and not occupied)))
    return finish('all_old_locations_supported' if all(p['clear'] for p in proofs) else 'old_location_unconfirmed',
        accepted=all(p['clear'] for p in proofs),fit=fit,proofs=proofs,corrected_points=old.tolist())


class PositiveClearanceTrial:
    """원래 위치/전체 점을1초 동안 보존하고 두 개의 새 스캔에서만 확인한다.

    원래 재관측 상태기의0.3초 만료를 수정하지 않는다. 별도의 명시 반환
    경로이며 시간 만료 자체는 해제가 되지 않는다. 현재 입력 재검사는 호출자 책임.
    """
    def __init__(self):self.reset()

    def reset(self):
        self.original=None
        self.last_reference=self.last_source=None
        self.consecutive=0
        self.ended=False

    def start(self,retained,points,ids,now,source):
        checked=SurfaceAnchorTrial();checked.start(retained,points,ids,now,source)
        self.reset()
        self.original=(checked.retained.copy(),checked.anchor[0].copy(),float(now))
        self.last_reference=float(now);self.last_source=float(source)

    def observe(self,scene,history,current,ids):
        def finish(reason,**extra):
            return dict(reason=reason,matched=False,consecutive_fresh_sources=self.consecutive,
                unknown_filled=False,**extra)
        if self.original is None or self.ended:return finish('inactive_or_ended')
        now,source=scene.t_ref_s,scene.source_first_s
        if (not source_valid(scene) or now<self.last_reference or source<self.last_source):
            self.ended=True;self.consecutive=0;return finish('invalid_or_reversed_time')
        if now-self.original[2]>1.:
            self.ended=True;self.consecutive=0;return finish('positive_history_expired')
        self.last_reference=now
        if source==self.last_source:return finish('same_scan_no_confirmation')
        self.last_source=source
        result=fresh_clearance(*self.original,scene,history,current,ids)
        self.consecutive=self.consecutive+1 if result['accepted'] else 0
        return {**result,'matched':self.consecutive>=2,'consecutive_fresh_sources':self.consecutive}


class RollingClearanceTrial(PositiveClearanceTrial):
    def reset(self):
        super().reset()
        self.rolling_anchor = None
        self.rolling_last_source = None
        self.rolling_consecutive = 0

    def bind(self, anchor):
        """처음 캡처한 바로 그 전체 점/배경/시각에만 연결한다."""
        if (not isinstance(anchor, SurfaceAnchorTrial) or self.original is None
                or anchor.anchor is None or anchor.ended
                or anchor.created != self.original[2]
                or not np.array_equal(anchor.retained, self.original[0])
                or not np.array_equal(anchor.anchor[0], self.original[1])):
            raise ValueError('rolling anchor must share the complete original evidence')
        self.rolling_anchor = anchor
        self.rolling_last_source = anchor.last_source

    def observe(self, scene, history, current, ids):
        original = super().observe(scene, history, current, ids)
        if original.get('matched') or self.ended or self.original is None:
            self.rolling_consecutive = 0
            return original
        anchor = self.rolling_anchor
        if anchor is None:
            return original
        now, source = scene.t_ref_s, scene.source_first_s
        if source == self.rolling_last_source:
            return {**original, 'rolling_clearance':dict(reason='same_scan_no_confirmation',
                consecutive_fresh_sources=self.rolling_consecutive)}
        self.rolling_last_source = source
        points = np.asarray(current, dtype=float)
        # anchor.anchor는 적합을 승인했을 때만 현재 점군/시각으로 바뀐다.
        checked = (not anchor.ended and anchor.anchor is not None
            and anchor.created == self.original[2]
            and math.isfinite(now) and 0 <= now-anchor.created <= 1.
            and anchor.anchor[1] == now and anchor.anchor[2] == source
            and anchor.last_reference == now and anchor.last_source == source
            and anchor.retained.shape == self.original[0].shape
            and np.all(np.isfinite(anchor.retained))
            and anchor.anchor[0].shape == points.shape
            and np.array_equal(anchor.anchor[0], points))
        if not checked:
            self.rolling_consecutive = 0
            return {**original, 'rolling_clearance':dict(reason='current_complete_anchor_required',
                consecutive_fresh_sources=0)}
        proof = fresh_clearance(anchor.retained, points, now, scene, history, points, ids)
        self.rolling_consecutive = self.rolling_consecutive+1 if proof['accepted'] else 0
        proof = {**proof, 'consecutive_fresh_sources':self.rolling_consecutive,
                 'original_reference_s':self.original[2], 'current_reference_s':now,
                 'complete_points':len(anchor.retained), 'static_background_assumption':True}
        if self.rolling_consecutive < 2:
            return {**original, 'rolling_clearance':proof}
        return {**proof, 'reason':'rolling_old_locations_supported', 'matched':True,
                'original_clearance':original, 'rolling_clearance':proof}


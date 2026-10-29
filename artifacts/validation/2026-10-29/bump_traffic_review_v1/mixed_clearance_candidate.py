"""미연결 비교안: 점별 재관측/빈 위치 근거와 직전 실제 반환의 제한 결합.

두 판단의 전체 증거는 서로 독립이지 않다. 재사용한 광선의 취득 나이를
유지하고 2 m/s 여유를 다시 뺀다. 현재 중심 반환 없이 옛 자료만으로
빈 위치를 확인하지 않으며 원래 unknown/점유와 1초 기억 수명을 유지한다.
"""
from dataclasses import replace
import math
import numpy as np

from arena_autonomy.anchor_clearance_trial import RollingClearanceTrial
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.observation_support import depth_support, transport_scene, source_valid
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.surface_reobservation import supported_surface
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy


def disk_queries(point):
    grid=np.array([(x,y) for x in np.arange(-3,4)*.01 for y in np.arange(-3,4)*.01
        if x*x+y*y<=.03**2+1e-12])
    theta=np.arange(24)*math.tau/24
    return point+np.vstack((grid,.03*np.column_stack((np.cos(theta),np.sin(theta)))))


def aligned_previous(previous, current, retained, history):
    """동일 전체 점의 승인 정합만 사용하며 직전 장면의 실제 시각/상태를 보존."""
    if previous is None:return None
    old,old_points=previous
    if (old.source_epoch!=current.source_epoch or not source_valid(old)
            or not old.source_first_s<current.source_first_s
            or not 0<current.t_ref_s-old.source_first_s<.3
            or old_points.shape!=retained.shape):return None
    carried=transport_points(old_points,history,old.t_ref_s,current.t_ref_s)
    shifts=retained-carried
    if not np.all(np.isfinite(shifts)) or not np.allclose(shifts,shifts[0],rtol=0,atol=1e-8):
        return None
    shifted=transport_scene(old,history,current.t_ref_s)
    def point(p):return tuple(np.asarray(p)+shifts[0])
    return replace(shifted,rays=tuple(replace(r,ray_origin_xy_m=point(r.ray_origin_xy_m),
        hit_xy_m=None if r.hit_xy_m is None else point(r.hit_xy_m)) for r in shifted.rays))


def combine_points(scene, retained, points, ids, clear_proofs, previous):
    """현재 점유/재관측은 지우지 않는다. 반환이 빈 위치를 지지한 점만 분리."""
    distance=np.min(np.linalg.norm(retained[:,None]-points[None],axis=2),axis=1)
    observed=[bool(d<=.02 or (d<=.04 and supported_surface(p,points,ids) is not None))
              for p,d in zip(retained,distance)]
    padding=np.array([.03+2*(scene.t_ref_s-r.acquired_at_s) for r in scene.rays])
    old_padding=None if previous is None else np.array([
        .03+2*(scene.t_ref_s-r.acquired_at_s) for r in previous.rays])
    evidence=[]
    for point,seen,proof in zip(retained,observed,clear_proofs):
        detail=dict(observed=seen,current_clear=bool(proof['clear']),recent_clear=False,
            occupied=bool(proof['occupied']),accounted=seen or bool(proof['clear']))
        if not detail['accounted'] and not proof['occupied'] and previous is not None:
            queries=disk_queries(point)
            support=depth_support(scene,queries,padding)
            center=bool(depth_support(scene,[point],padding)[0])
            old_support=depth_support(previous,queries,old_padding)
            combined=support|old_support
            detail.update(current_center_supported=center,current_supported=int(support.sum()),
                previous_supported=int(old_support.sum()),combined_supported=int(combined.sum()),
                queries=len(queries),previous_age_s=scene.t_ref_s-previous.source_first_s,
                recent_clear=bool(center and np.all(combined)))
            detail['accounted']=detail['recent_clear']
        evidence.append(detail)
    return evidence


class MixedClearanceCandidate(RollingClearanceTrial):
    def reset(self):
        super().reset()
        self.mixed_previous=None
        self.mixed_last_source=None
        self.mixed_count=0
        self.mixed_epoch=None
        self.mixed_epoch_seen=False

    def observe(self,scene,history,current,ids):
        if self.mixed_epoch_seen and scene.source_epoch!=self.mixed_epoch:
            self.ended=True;self.mixed_count=0;self.mixed_previous=None
            return dict(matched=False,reason='mixed_epoch_changed',unknown_filled=False)
        self.mixed_epoch=scene.source_epoch;self.mixed_epoch_seen=True
        base=super().observe(scene,history,current,ids)
        def finish(reason,**extra):
            return {**base,'mixed_clearance':dict(reason=reason,
                consecutive_fresh_sources=self.mixed_count,unknown_filled=False,
                independent_complete_scans=False,**extra)}
        if base.get('matched') or self.ended or self.original is None:
            self.mixed_count=0;self.mixed_previous=None
            return base
        source=scene.source_first_s
        if source==self.mixed_last_source:return finish('same_scan_no_confirmation')
        self.mixed_last_source=source
        rolling=base.get('rolling_clearance') or {}
        proof=rolling.get('proofs')
        retained=np.asarray(rolling.get('corrected_points',[]),dtype=float)
        anchor=self.rolling_anchor
        if (anchor is None or proof is None or not len(proof)
                or retained.shape!=self.original[0].shape or len(proof)!=len(retained)
                or rolling.get('current_reference_s')!=scene.t_ref_s
                or not np.allclose(retained,anchor.retained,rtol=0,atol=1e-8)):
            self.mixed_count=0;self.mixed_previous=None
            return finish('current_complete_anchor_required')
        try:previous=aligned_previous(self.mixed_previous,scene,retained,history)
        except MotionUnavailable:previous=None
        evidence=combine_points(scene,retained,np.asarray(current),np.asarray(ids),proof,previous)
        self.mixed_previous=(scene,retained.copy())
        accounted=all(p['accounted'] for p in evidence)
        self.mixed_count=self.mixed_count+1 if accounted else 0
        detail=dict(reason='mixed_locations_accounted' if accounted else 'mixed_location_unconfirmed',
            consecutive_fresh_sources=self.mixed_count,evidence=evidence,unknown_filled=False,
            original_reference_s=self.original[2],current_reference_s=scene.t_ref_s,
            complete_points=len(retained),static_background_assumption=True,
            independent_complete_scans=False,residual_hidden_object_risk=True)
        if self.mixed_count<2:return {**base,'mixed_clearance':detail}
        return {**base,'reason':'mixed_locations_accounted','matched':True,'mixed_clearance':detail}


class MixedRecoveryCandidate(RecoveryPolicy):
    def reset(self):
        super().reset()
        self.clearance_probe=MixedClearanceCandidate()

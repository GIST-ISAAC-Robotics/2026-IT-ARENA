"""저장 입력 전용 재관측 추적기. 판단 허가나 속도/조향을 반환하지 않는다.

정적 배경이 충분하다는 가정으로 마지막 승인 스캔을 유지한다. 거절된 스캔을
새 기준으로 쓰지 않으며, 시각 공백/누적 시간 한도 뒤에는 미확인으로 끝낸다.
센서의 표면 대응은 물체 신원·안전 증명이 아니다.
"""
import math
import numpy as np
from arena_autonomy.partial_observation import short_observed_surface
from arena_autonomy.partial_surface_alignment import surface_translation
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.lidar_motion import MotionUnavailable


class SurfaceAnchorProbe:
    def __init__(self):
        self.reset()

    def reset(self):
        self.anchor=None
        self.retained=None
        self.created=None
        self.last_source=None
        self.last_reference=None
        self.consecutive=0
        self.ended=False

    @staticmethod
    def arrays(points, ids):
        points=np.asarray(points,dtype=float)
        ids=np.asarray(ids)
        if (points.ndim!=2 or points.shape[1]!=2 or len(points)<3 or len(points)>1024 or
                ids.shape!=(len(points),) or not np.issubdtype(ids.dtype,np.integer) or
                not np.all(np.isfinite(points)) or np.any(np.diff(ids)<=0)):
            raise ValueError('invalid observed surface')
        return points,ids

    def start(self, retained, points, ids, now, source):
        self.reset()
        current,ids=self.arrays(points,ids)
        retained=np.asarray(retained,dtype=float)
        if (retained.ndim!=2 or retained.shape[1]!=2 or not 1<=len(retained)<=1024 or
                not np.all(np.isfinite(retained)) or not math.isfinite(now) or
                not math.isfinite(source) or not 0<=now-source<.3):
            raise ValueError('invalid retained evidence')
        self.retained=retained.copy()
        self.anchor=(current.copy(),float(now),float(source))
        self.created=float(now);self.last_reference=float(now);self.last_source=float(source)

    def observe(self, points, ids, history, now, source):
        result=dict(scope='diagnostic_reobservation_only',matched=False,unknown_filled=False,
            static_background_assumption=True,consecutive_fresh_sources=self.consecutive)
        def finish(reason,**extra):
            return {**result,**extra,'reason':reason,'consecutive_fresh_sources':self.consecutive}
        if self.anchor is None or self.ended:
            return finish('inactive_or_ended')
        try:current,ids=self.arrays(points,ids)
        except (ValueError,TypeError):
            self.ended=True;self.consecutive=0;return finish('invalid_input')
        if (not math.isfinite(now) or not math.isfinite(source) or not 0<=now-source<.3 or
                now<self.last_reference or source<self.last_source):
            self.ended=True;self.consecutive=0;return finish('invalid_or_reversed_time')
        self.last_reference=now
        if now-self.created>1. or now-self.anchor[1]>.3:
            self.ended=True;self.consecutive=0;return finish('diagnostic_history_expired')
        if source==self.last_source:
            return finish('same_scan_no_confirmation')
        self.last_source=source
        old,stamp,_=self.anchor
        try:
            carried=transport_points(old,history,stamp,now)
            retained=transport_points(self.retained,history,stamp,now)
        except MotionUnavailable:
            self.ended=True;self.consecutive=0;return finish('motion_unavailable')
        fit=surface_translation(carried,current,ids)
        if not fit['accepted']:
            self.consecutive=0
            return finish('surface_rejected',fit=fit)
        shifted=retained+np.asarray(fit['translation_m'])
        distance=np.min(np.linalg.norm(shifted[:,None]-current[None],axis=2),axis=1)
        observed=np.array([d<=.02 or (d<=.04 and short_observed_surface(p,current,ids) is not None)
                           for p,d in zip(shifted,distance)],dtype=bool)
        self.consecutive=self.consecutive+1 if np.all(observed) else 0
        self.anchor=(current.copy(),now,source)
        self.retained=shifted.copy()
        return finish('reobserved' if np.all(observed) else 'retained_points_unobserved',
            matched=self.consecutive>=2,fit=fit,observed=observed.tolist(),
            corrected_points=shifted.tolist(),nearest_distances_m=distance.tolist())

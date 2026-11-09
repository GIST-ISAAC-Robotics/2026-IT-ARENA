"""저장 재생 전용: 먼 좁은 지주와 분리된 현재 근접 면의 기존 운동 예산 비교."""
from dataclasses import replace
import numpy as np
from arena_autonomy.bypass_corridor_trial import BoundaryMonitor, boundary_face, wall_frame
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.lidar_motion import MotionUnavailable


class NearbyFace:
    def __init__(self,policy):
        # 기존 뒤 관측 범위의 절댓값을 앞에도 사용한다. 미래 경로의 다른 반환은 그대로다.
        self.policy=replace(policy,along_max_m=abs(policy.along_min_m))
        self.monitor=BoundaryMonitor(self.policy)

    def apply(self,scene,config,side,wall,history,padding,growth_s=0.):
        p=self.policy
        points=np.asarray(scene.occupied_endpoints_xy_m).reshape(-1,2)
        rays=[r for r in scene.rays if r.status=='return']
        ids=np.asarray([r.source_index for r in rays])
        along,normal=wall_frame(points,config,side,wall)
        fixed=.025+config.measurement_padding_m
        # 기존 외벽/다른 면의 더 작은 여유는 증가시키지 않는다.
        face=boundary_face(ids,points,along,normal,padding>fixed+1e-12,p)

        def moved(row):
            if history is None or row.get('endpoints') is None:return None
            try:return transport_points(row['endpoints'],history,row['reference'],scene.t_ref_s)
            except (ValueError,MotionUnavailable):return None

        def to_current(row):
            previous=moved(row)
            if previous is None:return None
            values=wall_frame(previous,config,side,wall)[0]
            return float(values.min()),float(values.max())

        if face is not None:
            members=face['members']
            face['endpoints_xy_m']=points[[members[np.argmin(along[members])],members[np.argmax(along[members])]]]
            if self.monitor.rows and self.monitor.key!=(scene.source_epoch,scene.source_first_s):
                previous=moved(self.monitor.rows[-1])
                if previous is None or abs(float(np.median(wall_frame(previous,config,side,wall)[1]))-face['normal_m'])>2*p.noise_budget_m:
                    self.monitor.reset()
        state=self.monitor.update(scene,face,to_current)
        result=padding.copy()
        supported=np.zeros(len(points),bool)
        if face is not None:
            supported[face['members']]=True
            supported &= abs(normal-face['normal_m'])<=p.snap_budget_m
        if state.get('stable') and supported.any():
            age=np.array([scene.t_ref_s-r.acquired_at_s+growth_s for r in rays])
            reduced=fixed+state['observed_rate_term_mps']*age+.5*p.onset_acceleration_mps2*age*age
            result[supported]=np.minimum(result[supported],reduced[supported])
        return result,dict(state=state,changed=int(np.count_nonzero(result<padding-1e-12)),
            supported=int(supported.sum()),normal_m=None if face is None else face['normal_m'],
            along_m=None if face is None else list(face['along_range_m']),
            all_returns_retained=True,static_identity_claimed=False,unknown_filled=False)

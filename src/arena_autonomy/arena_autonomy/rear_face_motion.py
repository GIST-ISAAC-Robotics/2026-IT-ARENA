"""현재 후방 짧은 면의 연속 관측. 정적 벽/자유 공간으로 분류하지 않는 선택형 보완."""
from dataclasses import replace
import numpy as np
from arena_autonomy.bypass_corridor_trial import BoundaryMonitor,wall_frame
from arena_autonomy.known_structure import classify
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.wall_reference_support import observed_rear_wall_face


class RearFaceMotion:
    def __init__(self,policy):
        # 같은 끝점 오차를 더 긴 실제 관측 창에서 비교한다. 순간 속도의 물리 상계는 아니다.
        self.monitor=BoundaryMonitor(replace(policy,window_s=.8,minimum_scans=9))
        self.policy=policy

    def apply(self,scene,config,side,wall,scan,sectors,history,padding,growth_s=0.,*,active=False):
        face=None;proof=None;indices=np.array([],dtype=int)
        rays=[r for r in scene.rays if r.status=='return']
        if (active and scan is not None and sectors and len(scan.ranges)==len(scene.rays)
                and abs(scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9-scene.source_first_s)<1e-8
                and wall.get('reason')=='current_connected_straight_wall_fit'):
            _,_,rows=classify(scan,sectors)
            explained={frozenset(r['rays'].tolist()) for r in rows if r['explained']}
            bounds=wall.get('rear_geometry_witness',{}).get('principal_x_m',wall['observed_x_m'])
            fit=(wall['slope'],wall['intercept_m'],*bounds)
            observed=observed_rear_wall_face(scene,config,side,fit,wall,explained)
            if observed is not None:
                proof=observed['proof']
                points=np.asarray([r.hit_xy_m for r in rays])
                along,normal=wall_frame(points,config,side,wall)
                selected=np.isin([r.source_index for r in rays],observed['source_ray_indices'])
                selected&=(points[:,0]<=-config.length_m/2)&(abs(normal)<=.015)
                indices=np.flatnonzero(selected)
                if len(indices)>=12 and np.ptp(along[indices])>=.20:
                    ends=indices[[np.argmin(along[indices]),np.argmax(along[indices])]]
                    # 외곽 면의 안쪽 접근은 양의 normal이므로 기존 안쪽 면과 부호가 반대다.
                    face=dict(normal_m=-float(np.median(normal[indices])),
                        along_range_m=[float(min(along[indices])),float(max(along[indices]))],
                        endpoints_xy_m=points[ends])
        def to_current(row):
            if history is None or row['endpoints'] is None:return None
            try:moved=transport_points(row['endpoints'],history,row['reference'],scene.t_ref_s)
            except (ValueError,MotionUnavailable):return None
            along=wall_frame(moved,config,side,wall)[0]
            return float(min(along)),float(max(along))
        state=self.monitor.update(scene,face,to_current)
        changed=0
        if face is not None and state.get('stable'):
            ages=np.asarray([scene.t_ref_s-rays[i].acquired_at_s+growth_s for i in indices])
            reduced=(.025+config.measurement_padding_m+state['observed_rate_term_mps']*ages
                     +.5*self.policy.onset_acceleration_mps2*ages*ages)
            original=padding
            padding=padding.copy();padding[indices]=np.minimum(padding[indices],reduced)
            changed=int(np.sum(padding<original-1e-12))
        detail=dict(t=scene.t_ref_s,side=side,face=face is not None,stable=state.get('stable'),
            changed=changed,state=state,proof=proof,unknown_filled=False,static_identity_claimed=False,
            rear_indices=[rays[i].source_index for i in indices])
        return padding,detail

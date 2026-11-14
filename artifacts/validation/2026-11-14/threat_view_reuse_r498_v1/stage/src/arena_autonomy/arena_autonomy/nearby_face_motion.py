"""선택형 우회 중 가까운 관측 조각을 각각 추적한다.

먼 지주가 통로 계획의 대표 면이 되어도 현재 옆차의 관측 창을 잃지 않게 한다.
이미 선택된 rear-wall-witness 우회 후보에서만 사용하며, 고정 물체로 단정하거나
가림 구간을 연결하지 않는다. 후방 면과 같은 9스캔/.8초 창에서 기존 접근/잡음
예산·가속 가정을 각 조각에 따로 적용한다. 순간 접근 속도의 물리적 상계는 아니다.
"""
from dataclasses import replace
import numpy as np
from arena_autonomy.bypass_corridor_trial import BoundaryMonitor,boundary_face,wall_frame
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.lidar_motion import MotionUnavailable


class NearbyFragments:
    def __init__(self,policy):
        # 더 긴 실제 관측으로 같은 끝점 오차의 시간당 항을 계산한다.
        # 새 면/관측 소실은 다시 창을 채워야 하며, 현재 접근량 검사는 유지한다.
        self.policy=replace(policy,along_max_m=abs(policy.along_min_m),window_s=.8,minimum_scans=9)
        self.monitors=[BoundaryMonitor(self.policy) for _ in range(4)]
        self.key=None

    def apply(self,scene,config,side,wall,history,padding,growth_s=0.,*,active=True):
        if not active:
            for monitor in self.monitors:monitor.reset()
            self.key=None
            return padding,dict(changed=0,reason="inactive_bypass",unknown_filled=False)
        p=self.policy
        points=np.asarray(scene.occupied_endpoints_xy_m).reshape(-1,2)
        rays=[r for r in scene.rays if r.status=='return']
        ids=np.asarray([r.source_index for r in rays])
        along,normal=wall_frame(points,config,side,wall)
        fixed=.025+config.measurement_padding_m
        remaining=padding>fixed+1e-12
        faces=[]
        for _ in self.monitors:
            face=boundary_face(ids,points,along,normal,remaining,p)
            if face is None:break
            members=face['members'];remaining[members]=False
            face['endpoints_xy_m']=points[[members[np.argmin(along[members])],members[np.argmax(along[members])]]]
            faces.append(face)
        result=padding.copy(); evidence=[];used=set()
        key=(scene.source_epoch,scene.source_first_s)
        if self.key is not None and (key[0]!=self.key[0] or key[1]<self.key[1] or key[1]-self.key[1]>.25):
            for monitor in self.monitors:monitor.reset()

        def moved(row):
            if history is None or row.get('endpoints') is None:return None
            try:return transport_points(row['endpoints'],history,row['reference'],scene.t_ref_s)
            except (ValueError,MotionUnavailable):return None

        def to_current(row):
            previous=moved(row)
            if previous is None:return None
            v=wall_frame(previous,config,side,wall)[0]
            return float(v.min()),float(v.max())

        for face in sorted(faces,key=lambda f:f['along_range_m'][0]):
            candidates=[]
            for index,monitor in enumerate(self.monitors):
                if index in used or not monitor.rows:continue
                previous=moved(monitor.rows[-1])
                if previous is None:continue
                old_along,old_normal=wall_frame(previous,config,side,wall)
                lo,hi=face['along_range_m']
                gap=max(lo,float(old_along.min()))-min(hi,float(old_along.max()))
                difference=abs(float(np.median(old_normal))-face['normal_m'])
                if gap<=p.face_association_gap_m and difference<=2*p.noise_budget_m:
                    candidates.append((max(0.,gap)+difference,index))
            if candidates:index=min(candidates)[1]
            else:
                index=next(i for i in range(len(self.monitors)) if i not in used)
                self.monitors[index].reset()
            used.add(index);monitor=self.monitors[index]
            state=monitor.update(scene,face,to_current)
            selected=face['members'][abs(normal[face['members']]-face['normal_m'])<=p.snap_budget_m]
            changed=0
            if state.get('stable') and len(selected):
                age=np.array([scene.t_ref_s-rays[i].acquired_at_s+growth_s for i in selected])
                reduced=fixed+state['observed_rate_term_mps']*age+.5*p.onset_acceleration_mps2*age*age
                reduced=np.where(np.isfinite(age)&(age>=0.),reduced,result[selected])
                before=result[selected].copy()
                result[selected]=np.minimum(result[selected],reduced)
                changed=int(np.count_nonzero(result[selected]<before-1e-12))
            evidence.append(dict(state=state,changed=changed,supported=len(selected),
                normal_m=face['normal_m'],along_m=list(face['along_range_m']),
                source_indices=ids[selected].tolist(),slot=index))
        if key!=self.key:
            for index,monitor in enumerate(self.monitors):
                if index not in used:monitor.reset()
        self.key=key
        return result,dict(fragments=evidence,changed=int(np.count_nonzero(result<padding-1e-12)),
            all_returns_retained=True,static_identity_claimed=False,unknown_filled=False)

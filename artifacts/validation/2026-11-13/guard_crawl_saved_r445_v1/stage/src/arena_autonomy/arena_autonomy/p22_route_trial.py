"""P22 입구→관측된 통로 끝→본선 복귀. 선택형 단독 시험이며 교통 합류용이 아니다."""
import math
import numpy as np

from arena_autonomy.p22_trial import P22Entry, entry_command
from arena_autonomy.p22_exit_alignment import propose
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.lidar_motion import MotionUnavailable


def body_path(candidate, wheelbase):
    poses=np.asarray(candidate['poses'])
    return poses[:,:2]+wheelbase/2*np.column_stack((np.cos(poses[:,2]),np.sin(poses[:,2])))


def tail_heading(path):
    v=path[-1]-path[-3]
    return math.atan2(v[1],v[0])


class P22Route(P22Entry):
    def _hold(self,why):
        result=super()._hold(why)
        return result[0],result[1],dict(result[2],mode='p22_route_trial',p22_trial=dict(
            result[2]['p22_trial'],scope='limited_solo_entry_exit',phase=self.phase,
            merged_s=self.merged_at,main_progress_m=self.main_progress,route_complete=False))

    def reset(self, *, force=False):
        preserve=not force and (getattr(self,'path',None) is not None or getattr(self,'latched_fault',None))
        super().reset(force=force)
        if not preserve:
            self.phase=None
            self.phase_t=None
            self.exit_seen=None
            self.exit_scan=None
            self.merge_scans=0
            self.merge_scan=None
            self.merge_time=None
            self.merge_ready=False
            self.main_progress=0.
            self.main_time=None
            self.merged_at=None

    def _detail(self,output,first=False):
        if first:
            self.phase='entry'
            self.phase_t=self.start_t
        result=super()._detail(output,first)
        return result[0],result[1],dict(result[2],mode='p22_route_trial',p22_trial=dict(
            result[2]['p22_trial'],scope='limited_solo_entry_exit',phase=self.phase,
            phase_start_s=self.phase_t,merged_s=self.merged_at,main_progress_m=self.main_progress,
            route_complete=self.phase=='main' and self.main_progress>=1.2))

    def _exit_candidate(self,scene,history,previous):
        if scene.source_first_s==self.exit_scan:return None
        self.exit_scan=scene.source_first_s
        # 입구 회전이 남아 있는데 출구 직선으로 갈아타면 통로 안쪽을 가로지른다.
        if abs(tail_heading(self.path))>math.radians(8) or abs(previous)>.14:
            self.exit_seen=None
            return None
        candidates=propose(scene,previous,self.path)
        if len(candidates)!=1:
            self.exit_seen=None
            return None
        candidate=candidates[0]
        path=body_path(candidate,self.config.wheelbase_m)
        old=self.exit_seen
        self.exit_seen=(scene.t_ref_s,path[-3:].copy())
        if old is None or not .05<=scene.t_ref_s-old[0]<=.20:return None
        retained=transport_points(old[1],history,old[0],scene.t_ref_s)
        if np.linalg.norm(retained[-1]-path[-1])>.08:return None
        delta=tail_heading(retained)-tail_heading(path)
        if abs(math.atan2(math.sin(delta),math.cos(delta)))>math.radians(8):return None
        return path

    def _active(self,scene,history,speed,previous_steering,control_dt,*,raw_scan=None):
        now=scene.t_ref_s
        try:
            if self.phase=='main':
                (a,b),_=history.poses([self.main_time,now],now)
                self.main_progress+=max(0.,math.cos(a[2])*(b[0]-a[0])+math.sin(a[2])*(b[1]-a[1]))
                self.main_time=now
                out=self.baseline.command(scene,history,speed,'left',previous_steering,control_dt,raw_scan=raw_scan)
                return self._detail(out)
            if now-self.phase_t>(5. if self.phase=='entry' else 6.):
                return self._hold('route_phase_timeout')
            self.path=transport_points(self.path,history,self.path_t,now)
            self.path_t=now
            if self.phase=='entry':
                candidate=self._exit_candidate(scene,history,previous_steering)
                if candidate is not None:
                    out=entry_command(scene,candidate,speed,previous_steering,control_dt,self.config)
                    if out[0]>0:
                        self.path=candidate
                        self.phase,self.phase_t='exit',now
                        return self._detail(out)
            out=entry_command(scene,self.path,speed,previous_steering,control_dt,self.config)
            if self.phase=='exit' and out[2]['path_remaining_m']<.48 and abs(tail_heading(self.path))<math.radians(8):
                if not self.merge_ready:
                    self.baseline.reset()
                    self.merge_ready=True
                normal=self.baseline.command(scene,history,speed,'left',previous_steering,control_dt,raw_scan=raw_scan)
                if normal[0]<=0:
                    self.merge_scans=0
                    self.merge_time=None
                elif scene.source_first_s!=self.merge_scan:
                    self.merge_scan=scene.source_first_s
                    self.merge_scans=(self.merge_scans+1 if self.merge_time is not None
                                      and .05<=now-self.merge_time<=.20 else 1)
                    self.merge_time=now
                if self.merge_scans>=2 and normal[0]>0:
                    self.phase,self.phase_t='main',now
                    self.merged_at=self.main_time=now
                    return self._detail(normal)
            elif self.phase=='exit':
                self.merge_scans=0
                self.merge_time=None
            return self._detail(out)
        except (ValueError,MotionUnavailable) as exc:
            if self.phase=='main' or self.merge_ready:
                self.baseline.reset()
                self.merge_ready=False
                self.merge_scans=0
                self.merge_time=None
            return self._hold(str(exc))


"""기본 주행과 분리된 단독 P22 '진입만' 후보. 합류·교통·경기 자동 사용은 지원하지 않는다."""
import math
import numpy as np
from arena_autonomy.p22_observation import observe
from arena_autonomy.p22_intent import BranchIntent
from arena_autonomy.p22_path import turn
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.local_follow import pursue_path,forward_planning_speed
from arena_autonomy.local_path import swept_limit,braking_speed
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.pass_release_trial import lateral_config,SelectorConfig,motion_region,region_blocked
from arena_autonomy.lidar_motion import MotionUnavailable


def entry_command(scene,path,speed,previous,dt,cfg):
    if not all(math.isfinite(x) for x in (speed,previous,dt)) or not 0<dt<=.1:
        raise MotionUnavailable('entry_invalid_command')
    speed=forward_planning_speed(cfg,speed)
    if path.ndim!=2 or path.shape[1]!=2 or len(path)<3 or not np.all(np.isfinite(path)):
        raise MotionUnavailable('entry_invalid_path')
    if scene.source_first_s is None:
        raise MotionUnavailable('entry_empty_scan')
    age=scene.t_ref_s-scene.source_first_s
    if not 0<=age<.3:raise MotionUnavailable('entry_stale_scan')
    cap,steer,remaining=pursue_path(path,speed,cfg,age,speed_cap=.65)
    steer=float(np.clip(steer,previous-3*dt,previous+3*dt))
    points=np.asarray(scene.occupied_endpoints_xy_m)
    if not len(points):raise MotionUnavailable('entry_empty_returns')
    pad=per_ray_padding(scene,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
    gap=min(swept_limit(points,a,cfg.wheelbase_m,horizon=2.5,margin=pad,length=cfg.length_m,width=cfg.width_m)
            for a in (steer,previous))
    target=min(.65,cap,braking_speed(gap,age))
    region=motion_region(scene,speed,target,steer,previous,dt,cfg,lateral_config(cfg))
    reason=region_blocked(region,SelectorConfig()) or ('occupied' if target<=0 else 'candidate')
    return (target if reason=='candidate' else 0.,steer,
            dict(reason='p22_'+reason,behavior='BRANCH_ENTRY' if reason=='candidate' else 'HOLD',
                 path_remaining_m=remaining,command_clearance_m=gap,motion_region=region,
                 target_before_region_mps=target,mode='p22_entry_trial',simulation_trial_only=True))


class P22Entry:
    """기존 LocalFollow 바깥에서 명시 선택한다. 상태기 입력은 센서뿐이다."""
    def __init__(self, baseline):
        self.baseline,self.config=baseline,baseline.config
        if not (abs(self.config.wheelbase_m-.135)<1e-8 and abs(self.config.length_m-.20)<1e-8 and
                abs(self.config.width_m-.15)<1e-8):
            raise ValueError('p22 trial requires registered provisional geometry')
        self.reset(force=True)

    def reset(self, *, force=False):
        # 자동 센서 리셋 뒤 반쯤 진입한 자세에서 본선 조향으로 돌아가지 않는다.
        # 이 제한 시험의 경로 중단은 운영자 reset으로만 해제한다.
        if not force and (getattr(self, 'path', None) is not None or getattr(self, 'latched_fault', None)):
            self.latched_fault = self.latched_fault or 'active_input_reset'
            self.baseline.reset()
            return
        self.baseline.reset()
        self.intent=BranchIntent()
        self.path=self.path_t=self.start_t=None
        self.first_scan=self.observed_stamp=self.visual_epoch=None
        self.vision=None
        self.latched_fault=None

    def frame(self,ids,stamp,now,epoch):
        self.vision=dict(ids=list(ids),stamp=stamp,now=now,epoch=epoch)

    def command(self,scene,history,speed,side,previous_steering=0.,control_dt=.02,*,raw_scan=None):
        now=scene.t_ref_s
        if self.latched_fault:return self._hold(self.latched_fault)
        if self.first_scan is None:
            self.first_scan=now
            self.intent.reset(now,scene.source_epoch)
        if self.intent.epoch!=scene.source_epoch:
            had_path=self.path is not None
            self.reset(force=True);self.first_scan=now;self.intent.reset(now,scene.source_epoch)
            if had_path:
                self.latched_fault='epoch_reset'
                return self._hold(self.latched_fault)
        if self.vision is not None:
            v=self.vision
            if self.visual_epoch is not None and self.visual_epoch!=v['epoch']:
                self.intent.reset(now,scene.source_epoch)
                if self.path is not None:
                    self.latched_fault='vision_epoch'
                    return self._hold(self.latched_fault)
            self.visual_epoch=v['epoch']
            self.intent.frame(v['ids'],v['stamp'],now,scene.source_epoch)
        if self.path is None:
            normal=self.baseline.command(scene,history,speed,side,previous_steering,control_dt,raw_scan=raw_scan)
            if scene.source_first_s!=self.observed_stamp and self.intent.state=='armed':
                self.observed_stamp=scene.source_first_s
                motion=None
                if self.intent.last_time is not None:
                    try:
                        (a,b),_=history.poses([self.intent.last_time,now],now)
                        dx,dy=b[0]-a[0],b[1]-a[1];c,s=math.cos(a[2]),math.sin(a[2])
                        motion=(c*dx+s*dy,-s*dx+c*dy,b[2]-a[2])
                    except MotionUnavailable:pass
                notches=observe(scene)['right']['notches']
                if self.intent.scan(now,scene.source_first_s,notches,motion,scene.source_epoch):
                    # Pure Pursuit는 남은 전방 경로 .25 m에서 끝난다.
                    # 기존 정적 .25 m 꼬리를 .55 m까지 늘려 차체 전체의 입구 통과를 시험한다.
                    candidate=turn(notches[0],previous_steering,.36,.65,final_depth_m=.55)
                    if candidate['reason']=='candidate':
                        poses=np.asarray(candidate['poses'])
                        path=poses[:,:2]+self.config.wheelbase_m/2*np.column_stack((np.cos(poses[:,2]),np.sin(poses[:,2])))
                        try:
                            first=entry_command(scene,path,speed,previous_steering,control_dt,self.config)
                        except (ValueError,MotionUnavailable):
                            first=None
                        if first is not None and first[0]>0:
                            self.path,self.path_t,self.start_t=path,now,now
                            return self._detail(first,first=True)
            return normal[0],normal[1],dict(normal[2],p22_trial=dict(state=self.intent.state,reason=self.intent.reason,
                first_command_admitted=False))
        return self._active(scene,history,speed,previous_steering,control_dt,raw_scan=raw_scan)

    def _active(self,scene,history,speed,previous_steering,control_dt,*,raw_scan=None):
        now=scene.t_ref_s
        if now-self.start_t>5.:
            return self._hold('entry_window_complete')
        try:
            self.path=transport_points(self.path,history,self.path_t,now)
            self.path_t=now
            return self._detail(entry_command(scene,self.path,speed,previous_steering,control_dt,self.config))
        except (ValueError,MotionUnavailable) as e:
            return self._hold(str(e))

    def _detail(self,output,first=False):
        return output[0],output[1],dict(output[2],p22_trial=dict(state='active',first_command_admitted=first,
            start_s=self.start_t,path_t=self.path_t,scope='limited_entry_only',path_points=self.path.tolist()))

    def _hold(self,why):
        return 0.,0.,dict(reason='p22_'+why,behavior='HOLD',mode='p22_entry_trial',simulation_trial_only=True,
                          p22_trial=dict(state='active_hold',scope='limited_entry_only',start_s=self.start_t))

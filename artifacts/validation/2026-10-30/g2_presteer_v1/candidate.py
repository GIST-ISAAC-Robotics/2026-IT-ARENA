"""오프라인 G2 정상 대기 조향 후보. STOP/구동 허가와 분리된 명령 비교다."""
import math
import numpy as np
from arena_autonomy.bypass_entry_trial import EntryGuide,current_outer_wall_padding,entry_motion_region
from arena_autonomy.local_path import swept_limit,braking_speed
from arena_autonomy.observation_support import source_valid
from arena_autonomy.pass_release_trial import lateral_config,region_blocked,SelectorConfig
from arena_autonomy.m3_execution_trial import envelope_distance,ExecutionConfig


class SteeringRecoveryGuide(EntryGuide):
    def reset(self):
        super().reset()
        self.last_requested_speed=None

    def _record(self,result):
        chosen=result.get('selected')
        self.last_requested_speed=0. if chosen is None else chosen['speed_mps']
        return result

    def _angle(self,scene,points,padding,speed,previous,steer,dt):
        config=self.config;age=scene.t_ref_s-scene.source_first_s
        clearance=min(swept_limit(points,angle,config.wheelbase_m,horizon=2.5,
            margin=padding,length=config.length_m,width=config.width_m) for angle in {previous,steer})
        target=min(.10,self.entry.speed_cap_mps,braking_speed(clearance,age))
        detail=dict(steering_rad=steer,target_mps=target,clearance_m=clearance,
            speed_cap_mps=.10,reason='no_braking_space',eligible=False)
        if target<=0:return detail
        measured=envelope_distance(speed,age,dt,ExecutionConfig())+.06
        detail['measured_required_space_m']=measured
        if measured>clearance+1e-9:return dict(detail,reason='measured_stop_space_insufficient')
        region=entry_motion_region(scene,speed,target,steer,previous,dt,config,lateral_config(config),padding)
        blocked=region_blocked(region,SelectorConfig())
        return dict(detail,region=region,reason=blocked or 'current_observed_low_speed_arc',eligible=blocked is None)

    @staticmethod
    def _zero_feedback(history,now,speed):
        # 모터축 0은 차체 정지/서보 도달을 증명하지 않는다. 실제 차체 피드백으로
        # 바꿔 부르지 않으며, 이 오프라인 후보는 두 인접 0표본만 확인한다.
        if speed!=0.:return False
        stamps,values=history.wheels.available(now)
        return bool(len(stamps)>=2 and 0<=now-stamps[-1]<=history.config.max_extrapolation_s+1e-9
            and 0<stamps[-1]-stamps[-2]<=history.config.max_input_gap_s+1e-9
            and values[-1]==0. and values[-2]==0.)

    def evaluate(self,scene,history,speed,previous,dt):
        prior_zero=self.last_requested_speed==0.
        original=super().evaluate(scene,history,speed,previous,dt)
        if (original.get('selected') is not None or self.phase!='entry' or self.path is None
                or not self.stationary_outer_boundary or self.wall_confirmations<2
                or original.get('guide_mode')!='measured_motion_transport_current_outer_wall'
                or not source_valid(scene) or len(original.get('candidates',[]))!=1):
            return self._record(original)
        row=original['candidates'][0]
        if row['reason']!='known_occupied_command_hold':return self._record(original)
        points=np.asarray(scene.occupied_endpoints_xy_m,dtype=float).reshape(-1,2)
        padding,classification=current_outer_wall_padding(scene,self.config,self.side,original['reference']['walls'][self.side])
        # 이미 겹친 동적 팽창도 그대로 거절한다. 측방 비접근 예외를 연결하지 않는다.
        half=np.array([self.config.length_m/2,self.config.width_m/2])
        current_gap=np.max(np.abs(points)-half-padding[:,None],axis=1)
        if np.any(current_gap<=0.):return self._record(original)
        lo=max(-self.config.max_steering_rad,previous-3*dt)
        hi=min(self.config.max_steering_rad,previous+3*dt)
        desired=row['steering_rad']
        angles=sorted({lo,hi,previous,float(np.clip(0.,lo,hi)),desired})
        proposals=[self._angle(scene,points,padding,speed,previous,angle,dt) for angle in angles]
        available=[p for p in proposals if p['eligible']]
        diagnostic=dict(applied=False,kind='none',normal_zero_not_stop=True,current_overlap_ignored=False,
            point_padding_changed=False,previous_steering_checked=True,proposals=proposals,
            motor_zero_is_body_stationary_proof=False,servo_reached_verified=False,
            future_permission_remembered=False,full_bypass_verified=False)
        rank=lambda p:(math.floor(p['target_mps']/.01+1e-9),-abs(p['steering_rad']-desired),p['clearance_m'])
        if available:
            best=max(available,key=rank)
            diagnostic.update(applied=True,kind='low_speed_alternative')
            selected=dict(offset_m=self.offset,transition_m=self.transition,
                speed_mps=best['target_mps'],steering_rad=best['steering_rad'],ranking='current_arc_then_path_steering')
            return self._record(dict(original,selected=selected,reason='entry_low_speed_steering_alternative',steering_recovery=diagnostic))
        if not prior_zero or not self._zero_feedback(history,scene.t_ref_s,speed):
            diagnostic['reason']='zero_command_and_two_motor_zero_samples_required'
            return self._record(dict(original,steering_recovery=diagnostic))
        # 방향을 바꾼 뒤 다시 검사할 후보를 고른다. 이 미래 비교 결과로 지금
        # 양수 출력을 내지 않는다. 이번 출력은 0이며 다음 호출은 실제 입력으로 재검사한다.
        future=[self._angle(scene,points,padding,0.,angle,angle,dt) for angle in angles if abs(angle-previous)>1e-9]
        future=[p for p in future if p['eligible']]
        diagnostic['future_angle_candidates']=future
        for best in sorted(future,key=rank,reverse=True):
            region=entry_motion_region(scene,0.,0.,best['steering_rad'],previous,dt,
                self.config,lateral_config(self.config),padding)
            if region_blocked(region,SelectorConfig()) is not None:continue
            diagnostic.update(applied=True,kind='zero_speed_presteer',zero_command_region=region)
            selected=dict(offset_m=self.offset,transition_m=self.transition,speed_mps=0.,
                steering_rad=best['steering_rad'],ranking='normal_zero_presteer_then_recheck')
            return self._record(dict(original,selected=selected,reason='entry_zero_speed_presteer',steering_recovery=diagnostic))
        return self._record(dict(original,steering_recovery=diagnostic))

"""정지 출발 시 중앙 합류 대신 현재 방향 직진을 비교하는 미연결 관측기.

기본 OFF. 이미 확인된 G1 경로의 gap HOLD에만 적용하며 경계 상실은 대체하지
않는다. 기존 M3 직진 계획/명령의 관측·노면·측방 예산과 G1 점유/재개 거리를
모두 검사한다. 결과는 명령이 아니며 신호·독립 보호·MCU·주행 중 유지/복귀는
미연결이다. 모터축 0은 실제 차체 정지 증명이 아니다.
"""
from dataclasses import replace
import math
import numpy as np

from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.local_path import braking_speed
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.lateral_intent import ObservationGrid, ego_state, corridor_reference
from arena_autonomy.m3_execution_trial import (
    ExecutionConfig, ExecutionFrame, M3ExecutionFollow, plan_check,
    presteer_motion_quiet, straight_candidate, arc_checks, envelope_distance)
from arena_autonomy.observation_support import source_valid
from arena_autonomy.occupancy_timing import per_ray_padding


class StartupStraightCandidate:
    """현재 장면만으로 0.2 m/s 이하 후보를 확인. 주행 함수에서 호출하지 않는다."""
    def __init__(self, config=None, *, enabled=False, rounded_occupancy=False):
        if type(enabled) is not bool or type(rounded_occupancy) is not bool:
            raise ValueError('explicit bool selection required')
        cfg = config or FollowConfig(occupancy_timing_mode='per_ray_candidate')
        self.enabled = enabled
        self.rounded_occupancy = rounded_occupancy
        self.config = replace(cfg, max_speed_mps=min(.2,cfg.max_speed_mps))
        execution = ExecutionConfig(command_fallback=True, release_to_baseline=True,
                                    small_center_straight=True, straight_surface_commitment=True)
        self.checker = M3ExecutionFollow(self.config, execution, allow_simulation_trial=True)
        self.reset()

    def reset(self):
        self.key = self.last_source = self.last_ref = None
        self.confirmations = 0

    def _rounded_command(self,path,scene,baseline,padding,dt):
        """시제품 국소 대조. 기본 M3 명령 함수/운영 모듈을 monkeypatch하지 않는다."""
        from arena_autonomy.rounded_occupancy_candidate import rounded_swept_limit
        cfg=self.config; age=scene.t_ref_s-scene.source_first_s
        try:
            cap,desired,remaining=pursue_path(path,0.,cfg,max(age,baseline[2]['boundary_age_s']))
        except MotionUnavailable:
            return dict(accepted=False,reason='path_too_short')
        steering=float(np.clip(desired,-3*dt,3*dt))
        clearance=min(rounded_swept_limit(scene.occupied_endpoints_xy_m,a,cfg.wheelbase_m,
            horizon=2.5,margin=padding,length=cfg.length_m,width=cfg.width_m) for a in (0.,steering))
        available=max(0.,clearance-cfg.standstill_gap_m)
        target=min(self.checker.trial_speed_cap_mps,cap,braking_speed(available,age),available/cfg.headway_s)
        distance=envelope_distance(target,age,dt,self.checker.execution)
        envelope=arc_checks(scene,[],baseline[2]['path_points'],(0.,steering),distance,
                            self.checker.lateral_config,self.checker.execution)
        accepted=envelope['observation_ok'] and envelope['geometry_ok']
        return dict(accepted=accepted,reason=None if accepted else 'rounded_command_envelope_rejected',
                    clearance=clearance,target=target,steering=steering,cap=cap,
                    detail=dict(envelope=envelope,remaining=remaining,
                        occupancy_model='fixed_rectangle_plus_euclidean_disk_candidate',
                        euclidean_motion_bound_assumed=True))

    def evaluate(self, scene, history, baseline, measured_speed, previous_steering=0., dt=.02):
        out = dict(selected=False, reason='disabled', proposed_speed_mps=0.,
                   proposed_steering_rad=0., confirmations=0, driving_authorized=False,
                   runtime_connected=False, initial_stationary_comparison_only=True,
                   body_standstill_verified=False, hidden_future_risk_unresolved=True)
        if not self.enabled:
            self.reset()
            return out
        key = (scene.source_epoch,scene.source_frame_id)
        if self.key is not None and (key != self.key or scene.t_ref_s < self.last_ref or
                (scene.source_first_s is not None and self.last_source is not None and scene.source_first_s < self.last_source)):
            self.reset()
        fresh = scene.source_first_s is not None and (self.last_source is None or scene.source_first_s > self.last_source)
        self.key,self.last_source,self.last_ref = key,scene.source_first_s,scene.t_ref_s

        def reject(reason):
            self.confirmations = 0
            return {**out,'reason':reason}

        if not source_valid(scene):
            return reject('source_invalid')
        if (not all(math.isfinite(v) for v in (measured_speed,previous_steering,dt)) or
                measured_speed != 0. or previous_steering != 0. or not 0 < dt <= .2):
            return reject('stationary_straight_entry_required')
        if not presteer_motion_quiet(history,scene.t_ref_s):
            return reject('stationary_measurements_unconfirmed')
        if baseline[0] != 0. or baseline[2].get('reason') not in ('follow_gap_hold','follow_release_wait'):
            return reject('baseline_not_gap_hold')
        detail = baseline[2]
        if (not detail.get('path_points') or not isinstance(detail.get('boundary_age_s'),(int,float)) or
                not 0 <= detail['boundary_age_s'] < self.config.memory_ttl_s):
            return reject('confirmed_path_required')
        if any(r.status != 'return' for r in scene.rays
               if abs(math.atan2(r.direction_xy[1],r.direction_xy[0])) <= math.radians(100)):
            return reject('front_unobserved')
        frame = ExecutionFrame(detail['path_points'])
        cfg,execution = self.checker.lateral_config,self.checker.execution
        state = ego_state(frame,cfg)
        straight = straight_candidate(frame,state,cfg)
        grid = ObservationGrid(scene,cfg)
        corridor = corridor_reference(frame,grid.returns,state['s0']-cfg.band_behind_m,state['s0']+cfg.horizon_m,cfg)
        padding = per_ray_padding(scene,.025+self.config.measurement_padding_m,self.config.object_motion_bound_mps)
        context = dict(frame=frame,grid=grid,corridor=corridor,age=scene.t_ref_s-scene.source_first_s,
                       follow_padding=padding)
        # frames=[]: 과거 관측·위협 기록을 가정해 만들어 넣지 않는다.
        plan = plan_check(scene,[],straight,context,cfg,execution,self.config.max_speed_mps)
        out['plan'] = plan
        if not plan['eligible_for_trial']:
            return reject('straight_plan_rejected')
        command = (self._rounded_command(straight['path'],scene,baseline,padding,dt) if self.rounded_occupancy else
                   self.checker._evaluate_command('straight',straight['path'],scene,[],history,0.,0.,dt,padding,baseline))
        out['command'] = command
        if not command['accepted'] or command['reason'] == 'm3_straight_presteer':
            return reject(command['reason'] or 'straight_command_rejected')
        if command['clearance'] < self.config.resume_clearance_m or command['target'] <= 0.:
            return reject('existing_resume_clearance_required')
        if fresh:
            self.confirmations += 1
        out['confirmations'] = self.confirmations
        if self.confirmations < self.config.confirmations:
            return {**out,'reason':'distinct_scan_confirmation_wait'}
        return {**out,'reason':'stationary_straight_candidate_only','selected':True,
                'proposed_speed_mps':command['target'],'proposed_steering_rad':command['steering']}

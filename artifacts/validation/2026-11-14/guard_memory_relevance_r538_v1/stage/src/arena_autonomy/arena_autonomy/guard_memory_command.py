"""소실 래치의 원 기억을 보존하는 명령별 저속 검사.

기억점은 자차 운동만큼 운반한다. 소실 물체가 독립적으로 움직이는2m/s 전체
도달집합 보장이 아니며, unknown이나 옛 점을 지우지 않는 명시 운용 정책이다.
"""
import math
import numpy as np
from arena_autonomy.independent_guard_route import front_evidence
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_path import swept_limit, braking_speed
from arena_autonomy.scan_input import scan_first_stamp
from arena_autonomy.scene_motion import transport_points, motion_scene
from arena_autonomy.partial_observation import short_observed_surface
from arena_autonomy.observation_support import depth_support, source_valid


def current_location_clearance(points, scene, current, indices):
    """운반된 옛 위치 주변을 현재 실제 반환으로 검사한다. 물체 신원 증명은 아니다.

    anchor_clearance_trial의30mm 주변 표본/반환 여유와 현재 광선의2m/s
    취득 지연 팽창을 재사용한다. 옛 probe의1초 기한/배경 직선 fit은 쓰지 않는다.
    """
    clear=np.zeros(len(points),dtype=bool)
    report=dict(accepted=False,reason='current_ray_support_unconfirmed',
        points=len(points),supported_centers=0,cleared_points=0,
        coordinate_radius_m=.03,return_margin_m=.03,motion_bound_mps=2.,
        source_first_s=scene.source_first_s,reference_s=scene.t_ref_s,
        unknown_filled=False,identity_proven=False)
    if not source_valid(scene):return clear,{**report,'reason':'current_ray_source_invalid'}
    expected=np.array([r.source_index for r in scene.rays if r.status=='return'])
    hits=np.asarray([r.hit_xy_m for r in scene.rays if r.status=='return']).reshape(-1,2)
    if (not np.array_equal(indices,expected) or hits.shape!=current.shape
            or not np.allclose(current,hits,rtol=0,atol=1e-8)):
        return clear,{**report,'reason':'current_ray_point_mismatch'}
    padding=np.array([.03+2*(scene.t_ref_s-r.acquired_at_s) for r in scene.rays])
    centers=depth_support(scene,points,padding)
    report['supported_centers']=int(centers.sum())
    # 중심 자체가 unknown/가림이면 주변 전체 검사는 불필요하다. 표본/원점은 불변이다.
    grid=np.array([(x,y) for x in np.arange(-3,4)*.01 for y in np.arange(-3,4)*.01
        if x*x+y*y<=.03**2+1e-12])
    theta=np.arange(24)*math.tau/24
    offsets=np.vstack((grid,.03*np.column_stack((np.cos(theta),np.sin(theta)))))
    candidates=np.flatnonzero(centers)
    if len(candidates):
        occupied=np.any(np.linalg.norm(points[candidates,None,:]-hits[None,:,:],axis=2)
                        <=padding[expected]+.03,axis=1)
        candidates=candidates[~occupied]
        queries=(points[candidates,None,:]+offsets).reshape(-1,2)
        if len(candidates):
            clear[candidates]=np.all(depth_support(scene,queries,padding).reshape(len(candidates),-1),axis=1)
    report.update(accepted=bool(np.any(clear)),cleared_points=int(clear.sum()),
        reason='current_finite_rays_pass_old_location' if np.any(clear) else 'current_ray_support_unconfirmed')
    return clear,report


class RetainedCommandMemory:
    """원 관측점과 현재 자차좌표 복사본. 물체 속도/신원 추정은 하지 않는다."""
    def __init__(self, points, reference, source, source_key, source_indices):
        points=np.asarray(points,dtype=float)
        if (points.ndim!=2 or points.shape[1]!=2 or not 1<=len(points)<=1024
                or not np.all(np.isfinite(points)) or not math.isfinite(reference)
                or not math.isfinite(source) or not 0<=reference-source<.3
                or not isinstance(source_key,tuple) or len(source_key)!=2
                or len(source_indices)!=len(points)):
            raise ValueError('invalid complete retained memory')
        self.original=points.copy();self.original.setflags(write=False)
        self.original_reference=float(reference);self.original_source=float(source)
        self.points=points.copy();self.reference=float(reference);self.last_source=float(source)
        self.source_key=source_key;self.motion_key=None;self.fault=None
        self.seen={(float(source),int(index)) for index in source_indices}
        self.pending=[];self.additional_original=[]
        self.reobserved_total=0
        self.ray_cleared_total=0
        self.last_ray_source=None

    def retain(self, points, reference, source, source_key, source_indices):
        """래치 중 새로 소실된 실제 반환도 보존한다. 동일 scan/index만 중복 제외한다."""
        if self.fault:return
        points=np.asarray(points,dtype=float)
        if (source_key!=self.source_key or points.shape!=(len(source_indices),2)
                or not np.all(np.isfinite(points)) or not math.isfinite(reference)
                or not math.isfinite(source) or not 0<=reference-source<.3):
            self.fault='retained_memory_addition_invalid';return
        fresh=[i for i,index in enumerate(source_indices) if (float(source),int(index)) not in self.seen]
        if not fresh:return
        addition=points[fresh].copy();addition.setflags(write=False)
        record=(addition,float(reference),float(source),tuple(int(source_indices[i]) for i in fresh))
        self.additional_original.append(record);self.pending.append(record)
        self.seen.update((float(source),index) for index in record[3])
        if len(self.points)+sum(len(row[0]) for row in self.pending)>1024:
            self.fault='retained_memory_active_capacity_exceeded'

    def advance(self, scan, history, now):
        if self.fault:raise MotionUnavailable(self.fault)
        source=scan_first_stamp(scan)
        key=(scan.header.frame_id,getattr(scan,'source_epoch',0))
        motion_key=(id(history.wheels),id(history.gyro))
        try:
            if key!=self.source_key:raise MotionUnavailable('retained_memory_epoch_changed')
            if self.motion_key is not None and motion_key!=self.motion_key:
                raise MotionUnavailable('retained_memory_motion_reset')
            if (not math.isfinite(now) or not 0<=now-source<.3
                    or now<self.reference or source<self.last_source):
                raise MotionUnavailable('retained_memory_time_invalid')
            carried=transport_points(self.points,history,self.reference,now)
            if carried.shape!=self.points.shape or not np.all(np.isfinite(carried)):
                raise MotionUnavailable('retained_memory_transport_invalid')
            if self.pending:
                carried=np.vstack([carried]+[transport_points(row[0],history,row[1],now) for row in self.pending])
                if not np.all(np.isfinite(carried)):raise MotionUnavailable('retained_memory_transport_invalid')
        except (ValueError,TypeError) as error:
            self.fault=str(error);raise MotionUnavailable(self.fault) from error
        self.points=carried;self.reference=float(now);self.last_source=float(source)
        self.pending=[]
        self.motion_key=motion_key
        return carried

    def observed_handoff(self, current, indices):
        """기존 재관측 근거에 한해 활성 계산만 현재 raw 점유로 인계한다. 원본은 보존한다."""
        if not len(self.points):return 0
        distances=np.min(np.linalg.norm(self.points[:,None,:]-current[None,:,:],axis=2),axis=1)
        observed=distances<=.02
        for index in np.flatnonzero((distances>.02)&(distances<=.04)):
            observed[index]=short_observed_surface(self.points[index],current,indices) is not None
        count=int(np.count_nonzero(observed))
        self.points=self.points[~observed]
        self.reobserved_total+=count
        return count

    def ray_handoff(self, scene, current, indices):
        """실제 광선의 옛 위치 통과 근거만 활성 계산에 반영한다. 원증거는 보존한다."""
        clear,report=current_location_clearance(self.points,scene,current,indices)
        self.points=self.points[~clear]
        self.ray_cleared_total+=int(clear.sum())
        self.last_ray_source=scene.source_first_s
        return report


def limited_memory_command(policy, partial, scan, points, history, now, requested, measured,
                           steering, previous, wheelbase, length, width):
    """선택된 독립 guard의 특정 래치만 다룬다. 실제 출력/래치 해제는 하지 않는다."""
    result=dict(accepted=False,policy='retained_memory_command_crawl_v1',speed_cap_mps=0.,
        unknown_filled=False,memory_deleted=False,latch_cleared=False,
        unresolved_dynamic_motion=True,full_2mps_reachable_set_guaranteed=False)
    def finish(reason,**extra):return {**result,**extra,'reason':reason}
    if (getattr(policy,'command_observation_priority',False) is not True
            or not policy.latched or policy.latch_reason!='previous_point_in_missing_sweep'
            or partial.get('allowed') is not False or partial.get('reason')!='known_obstacle_latched'):
        return finish('memory_command_not_eligible')
    memory=getattr(policy,'guard_memory',None)
    if not isinstance(memory,RetainedCommandMemory):return finish('complete_memory_unavailable')
    numbers=(now,requested,measured,steering,previous,wheelbase,length,width)
    if (not all(math.isfinite(v) for v in numbers) or requested<0
            or min(wheelbase,length,width)<=0 or max(abs(steering),abs(previous))>.6):
        return finish('memory_command_invalid')
    try:
        evidence=front_evidence(scan,history.config.lidar_yaw_rad)
        if not evidence['accepted']:return finish(evidence['reason'])
        current=np.asarray(points,dtype=float).reshape(-1,2)
        if not 30<=len(current)<=1024 or not np.all(np.isfinite(current)):
            return finish('memory_command_invalid_points')
        retained=memory.advance(scan,history,now)
        values=np.asarray(scan.ranges,dtype=float)
        ids=np.flatnonzero(np.isfinite(values)&(values>=scan.range_min)&(values<=scan.range_max))
        if len(ids)!=len(current):return finish('memory_command_point_order_mismatch')
        handed_off=memory.observed_handoff(current,ids)
        ray_report=None
        if len(memory.points) and memory.last_ray_source!=scan_first_stamp(scan):
            scene=motion_scene(scan,history,now,
                timing_verified=history.scan_time_policy=='verified_only',rear_half_angle_deg=0.)
            ray_report=memory.ray_handoff(scene,current,ids)
        elif len(memory.points):
            # 같은 scan의 반복 callback은 새 관측이 아니다. 운반/원점유 검사는 계속한다.
            ray_report=dict(accepted=False,reason='same_scan_no_new_ray_evidence',cleared_points=0)
        retained=memory.points
        result.update(retained_points=len(retained),original_reference_s=memory.original_reference,
            memory_reference_s=memory.reference,measured_speed_mps=measured,
            requested_speed_mps=requested,memory_retained=True,
            active_points_reobserved=handed_off,original_observation_points=len(memory.seen),
            active_points_ray_cleared=0 if ray_report is None else ray_report['cleared_points'],
            current_ray_clearance=ray_report)
        if requested==0:return finish('zero_command_preserved')
        age=now-scan_first_stamp(scan)
        all_reobserved=not len(retained)
        target=requested
        # 기반 래치가0이면 StructureAwareMixin은 cap을 partial 최상위에 합치지 않는다.
        # 새 명령 허가도 기존 실제 구조 기억의 독립 상한을 우회하지 않는다.
        for existing in (partial.get('speed_cap_mps'),(partial.get('known_structure') or {}).get('speed_cap_mps')):
            if existing is not None:
                if not math.isfinite(existing) or existing<0:return finish('existing_guard_cap_invalid')
                target=min(target,existing)
        if target<=0:return finish('existing_guard_cap_zero')
        checked_speed=max(abs(measured),target)
        # 기존 braking_speed의2m/s²·반응시간·60mm 여유와4cm 표본 간격을 재사용한다.
        horizon=max(.20,.06+checked_speed*(age+.10)+checked_speed**2/4+.04)
        # 비정상 실측을 무한 크기의 외곽 배열로 만들지 않는다. 기존 입력 오류로 유지한다.
        if horizon>3.:return finish('memory_command_braking_horizon_unavailable')
        angles=(steering,previous)
        memory_clearances=[swept_limit(retained,a,wheelbase,horizon=horizon,
            length=length,width=width) if len(retained) else horizon for a in angles]
        raw_clearances=[swept_limit(current,a,wheelbase,horizon=horizon,
            length=length,width=width) for a in angles]
        memory_relevant=any(value<horizon-1e-10 for value in memory_clearances)
        if not all_reobserved and memory_relevant:
            target=min(.10,target)
        result.update(memory_relevant_to_command=memory_relevant,
            relevance_scope='current and previous command stopping sweeps; retained points preserved')
        clearance=min(memory_clearances+raw_clearances)
        available_speed=braking_speed(clearance,age)
        details=dict(current_memory_clearance_m=memory_clearances[0],
            previous_memory_clearance_m=memory_clearances[1],current_raw_clearance_m=raw_clearances[0],
            previous_raw_clearance_m=raw_clearances[1],braking_speed_mps=available_speed,
            checked_speed_mps=checked_speed,horizon_m=horizon,current_angle_rad=steering,
            previous_angle_rad=previous)
        if available_speed<=0 or abs(measured)>available_speed+1e-10:
            return finish('memory_command_braking_clearance_insufficient',**details)
        cap=min(target,available_speed)
        if cap<=0:return finish('memory_command_no_positive_cap',**details)
        resolved_reason=('retained_memory_all_currently_observed' if memory.ray_cleared_total
                         else 'retained_memory_all_reobserved')
        return finish(resolved_reason if all_reobserved else
            ('retained_memory_current_sweep_crawl' if memory_relevant else 'retained_memory_outside_command_sweeps'),
            accepted=True,speed_cap_mps=cap,normal_cap_restored=all_reobserved or not memory_relevant,
            all_retained_points_reobserved=all_reobserved and not memory.ray_cleared_total,
            all_retained_points_currently_resolved=all_reobserved,**details)
    except (ValueError,TypeError,MotionUnavailable) as error:
        return finish('memory_command_input_failure',error=str(error))

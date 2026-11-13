"""고정 거리 접두 표본에서 한 개 저속 목표를 제안한다. 허가는 최종 기존 검사에만 있다."""
import math
import time

import numpy as np
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.observation_support import source_valid

DISTANCE_STEP_M = .01  # 기존 command_arc의 1cm 자세 표본 간격. 새 보호 판정선이 아니다.
MINIMUM_QUERY_DISTANCE_M = .01  # 기존 motion_queries의 최솟값.


def elapsed(costs,name,start):
    if costs is not None:costs[name]=(time.perf_counter_ns()-start)/1e6


def fixed_distances(distance,steerings,cfg):
    """원호 길이에 대해 고정된 1cm점과 기존 차체 x/y 극점, 한 끝점의 합집합."""
    values={0.,float(distance)}
    values.update(i*DISTANCE_STEP_M for i in range(1,math.floor(distance/DISTANCE_STEP_M)+1))
    for steering in steerings:
        k=math.tan(steering)/cfg.wheelbase_m
        if abs(k)<1e-9:continue
        lo,hi=sorted((0.,distance*k))
        for x in (-cfg.length_m/2,cfg.length_m/2):
            for y in (-cfg.width_m/2,cfg.width_m/2):
                a,b=1/k-y,cfg.wheelbase_m/2+x
                for critical in (math.atan2(a,b),math.atan2(-b,a)):
                    for n in range(math.ceil((lo-critical)/math.pi),math.floor((hi-critical)/math.pi)+1):
                        s=(critical+n*math.pi)/k
                        if 0<=s<=distance:values.add(float(s))
    return np.array(sorted(values),dtype=float)


def poses_at_distances(steering,distances,cfg):
    """기존 후륜축 정곡률 적분식이며 linspace를 재생성하지 않는다."""
    k=math.tan(steering)/cfg.wheelbase_m
    yaw=k*distances
    if abs(k)<1e-9:
        rx,ry=distances-cfg.wheelbase_m/2,np.zeros_like(distances)
    else:
        rx=np.sin(yaw)/k-cfg.wheelbase_m/2
        ry=(1-np.cos(yaw))/k
    return np.column_stack((rx+cfg.wheelbase_m/2*np.cos(yaw),ry+cfg.wheelbase_m/2*np.sin(yaw),yaw))


def observed_prefix(queries,first_distance,distances,free,masked,length,width,selector):
    """관측한 질의는 제약하지 않는다. unknown/자체마스크의 기존 기하 제한만 제안에 쓴다."""
    q=np.asarray(queries,dtype=float).reshape(-1,2)
    assert len(q)==len(first_distance)==len(free)==len(masked)
    unknown=~free & ~masked
    hidden=~free & masked
    exterior=np.abs(q[:,1])-width/2
    unknown_bad=unknown & ((np.abs(q[:,0])>length/2) | (exterior<=0) | (exterior>.01))
    masked_extent=np.maximum(exterior,-q[:,0]-length/2)
    masked_bad=hidden & ((q[:,0]>length/2) | (masked_extent>selector.masked_tail_allowance_m))
    bad=unknown_bad | masked_bad
    first=float(np.min(first_distance[bad])) if np.any(bad) else None
    preceding=distances if first is None else distances[distances<first]
    allowed=float(preceding[-1]) if len(preceding) else 0.
    return dict(allowed_distance_m=allowed,first_bad_distance_m=first,
        unknown_geometry_bad_queries=int(np.sum(unknown_bad)),masked_geometry_bad_queries=int(np.sum(masked_bad)),
        queries=len(q),free_queries=int(np.sum(free)),unknown_queries=int(np.sum(unknown)),masked_queries=int(np.sum(hidden)),
        maximum_exterior_m=.01,maximum_masked_tail_m=selector.masked_tail_allowance_m,
        distances_m=distances.tolist(),proposal_only=True,continuous_sweep_certified=False,
        unknown_filled=False,final_full_check_required=True)


def speed_from_distance(allowed,measured,original_target,tau,execution):
    """같은 D(v)를 안정적인 이차식으로 역산한다. 실제속도의 제동거리를 제거하지 않는다."""
    measured_distance=measured*tau+measured*measured/(2*execution.deceleration_mps2)
    result=dict(accepted=False,reason='distance_below_minimum_query',allowed_distance_m=allowed,
        measured_distance_m=measured_distance,minimum_query_distance_m=MINIMUM_QUERY_DISTANCE_M,
        tau_s=tau,reaction_s=execution.reaction_s,deceleration_mps2=execution.deceleration_mps2)
    if not math.isfinite(allowed) or allowed<MINIMUM_QUERY_DISTANCE_M:return result
    if measured_distance>allowed:return {**result,'reason':'measured_braking_distance_exceeds_budget'}
    speed=2*allowed/(tau+math.sqrt(tau*tau+2*allowed/execution.deceleration_mps2))
    # 한 번의 안쪽 반올림만 한다. 새 속도열이나 반복 탐색은 만들지 않는다.
    target=math.nextafter(min(float(original_target),.10,speed),0.)
    prediction=max(measured,target)
    distance=prediction*tau+prediction*prediction/(2*execution.deceleration_mps2)
    if not 0<target<=.10 or distance>allowed:
        return {**result,'reason':'inverse_rounding_or_target_invalid','proposed_target_mps':target,'proposal_distance_m':distance}
    return {**result,'accepted':True,'reason':'single_distance_budget_proposal',
        'proposed_target_mps':target,'proposal_distance_m':distance}


def propose(scene,measured,steering,previous,dt,cfg,*,target_cap=.10,costs=None):
    start=time.perf_counter_ns()
    if not source_valid(scene):return dict(accepted=False,reason='invalid_source')
    age=scene.t_ref_s-scene.source_first_s
    if not all(math.isfinite(v) for v in (measured,steering,previous,dt,age,target_cap)):
        return dict(accepted=False,reason='invalid_budget_input')
    execution=ps.mx.ExecutionConfig()
    assert execution.reaction_s==.10 and execution.deceleration_mps2==2.
    tau=dt+max(0.,age)+execution.reaction_s
    distance=ps.mx.envelope_distance(max(measured,target_cap),age,dt,execution)
    lateral=ps.lateral_config(cfg)
    steerings=sorted({float(steering),float(previous)})
    distances=fixed_distances(max(distance,MINIMUM_QUERY_DISTANCE_M),steerings,lateral)
    points=[];provenance=[];poses_count=0;thin_intersections=0
    for angle in steerings:
        poses=poses_at_distances(angle,distances,lateral)
        for s,pose in zip(distances,poses):
            # 기존 body_queries가 차체/바퀴/얇은 unknown 교차를 만든다. 발생 거리만 붙인다.
            q,thin=ps.mx.body_queries(scene,pose.reshape(1,3),np.array([angle]),lateral)
            q=q[(np.abs(q[:,0])>cfg.length_m/2+1e-9) | (np.abs(q[:,1])>cfg.width_m/2+1e-9)]
            points.append(q);provenance.append(np.full(len(q),s));poses_count+=1;thin_intersections+=thin
    all_queries=np.vstack(points);all_distances=np.concatenate(provenance)
    if not len(all_queries):return dict(accepted=False,reason='empty_proposal_queries')
    unique,inverse=np.unique(all_queries,axis=0,return_inverse=True)
    first_distance=np.full(len(unique),np.inf)
    np.minimum.at(first_distance,inverse,all_distances)
    elapsed(costs,'proposal_geometry_ms',start)
    start=time.perf_counter_ns()
    padding=ps.mx.ray_padding(scene,lateral,cfg.object_motion_bound_mps)
    classified=ps.depth_and_masked_support(scene,unique,padding)
    if classified is None:
        free=ps.depth_support(scene,unique,padding)
        masked=np.zeros(len(unique),dtype=bool)
        masked[~free]=ps.mx.masked_only(scene,unique[~free],padding)
    else:free,masked=classified
    elapsed(costs,'proposal_classification_ms',start)
    prefix=observed_prefix(unique,first_distance,distances,free,masked,cfg.length_m,cfg.width_m,ps.SelectorConfig())
    inverse_result=speed_from_distance(prefix['allowed_distance_m'],measured,target_cap,tau,execution)
    return {**inverse_result,'prefix':prefix,'cap_distance_m':distance,'target_cap_mps':target_cap,
        'current_steering_rad':steering,'previous_steering_rad':previous,'measured_speed_mps':measured,
        'query_pose_count':poses_count,'queries_before_exact_dedup':len(all_queries),
        'thin_intersections':int(thin_intersections),'source_first_s':scene.source_first_s,
        'actual_lateral_geometry':dict(length_m=lateral.length_m,width_m=lateral.width_m,wheelbase_m=lateral.wheelbase_m,
            track_m=lateral.track_m,wheel_radius_m=lateral.wheel_radius_m,wheel_width_m=lateral.wheel_width_m),
        'geometry_source':'same ps.lateral_config(cfg) and wheel_contact_corners as frozen baseline'}

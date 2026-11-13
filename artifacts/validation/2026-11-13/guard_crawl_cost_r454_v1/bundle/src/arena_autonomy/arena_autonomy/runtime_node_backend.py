"""선택형 두 계산 노드 연결. 기존 발행·신호·독립 보호를 통과한 결과만 확인한다."""
import math
from arena_autonomy.crop_edge_selection import (SIMULATION_SCOPE, ESTIMATED_SCOPE,
    validate_observation_scope, estimated_environment)

POLICY = 'cad_runtime_r377_v1'
CORNER_POLICY = 'r396_v1'


def validate_corner_selection(policy, *, runtime_selected, use_sim_time,
                              motion_verified, motion_time_policy, observation_scope=SIMULATION_SCOPE,
                              execution_environment=None, lidar_input_representation=None, motion_mode=None):
    validate_observation_scope(observation_scope)
    if policy == 'off':
        if observation_scope == ESTIMATED_SCOPE:
            raise ValueError('estimated observation requires the registered current corner selection')
        return False
    if policy != CORNER_POLICY or type(policy) is not str:
        raise ValueError('registered current corner budget required')
    if observation_scope == ESTIMATED_SCOPE:
        if runtime_selected is not True or not estimated_environment(
                execution_environment=execution_environment, use_sim_time=use_sim_time,
                motion_verified=motion_verified, motion_time_policy=motion_time_policy,
                motion_mode=motion_mode, lidar_input_representation=lidar_input_representation):
            raise ValueError('current estimated observation requires explicit physical native two-slot runtime')
        return True
    if (runtime_selected is not True or use_sim_time is not True
            or motion_verified is not True or motion_time_policy != 'verified_only'):
        raise ValueError('current corner budget requires verified simulation and registered native two-slot runtime')
    return True


def observation_options(node):
    scope = validate_observation_scope(getattr(node, 'observation_input_scope', SIMULATION_SCOPE))
    if scope == SIMULATION_SCOPE:
        return {}
    from arena_autonomy.native_lidar_input import input_representation
    return dict(observation_scope=scope, execution_environment=node.execution_environment,
                lidar_input_representation=input_representation(node), motion_mode=node.motion.mode)


def evaluator_header(node, header):
    selected = dict(header, runtime_policy=POLICY, runtime_time_policy=node.motion.timing_policy)
    policy = getattr(node, 'current_corner_budget', 'off')
    options = observation_options(node)
    if validate_corner_selection(policy, runtime_selected=node.runtime_dual_selected,
            use_sim_time=node.get_parameter('use_sim_time').value,
            motion_verified=node.motion.verified, motion_time_policy=node.motion.timing_policy, **options):
        selected.update(current_corner_budget=policy, current_corner_environment=dict(
            use_sim_time=True, motion_timing_verified=True, backend='native2slot_v1'))
        if options:
            selected['observation_input_scope'] = options['observation_scope']
            selected['current_corner_environment'] = dict(
                use_sim_time=node.get_parameter('use_sim_time').value,
                motion_timing_verified=node.motion.verified, backend='native2slot_v1',
                **{k: v for k, v in options.items() if k != 'observation_scope'})
    return selected


def validate_selection(policy, *, execution, submission, recheck_in_worker,
                       retain_reference, numeric_execution, hardware_shadow):
    if policy == 'off':
        return False
    if policy != POLICY:
        raise ValueError('registered runtime policy required')
    if (execution != 'process_candidate' or submission != 'fresh_native_v1'
            or recheck_in_worker is not True or retain_reference is not True
            or numeric_execution != 'native_v1' or hardware_shadow):
        raise ValueError('runtime policy requires native dual current checks and retained reference; no shadow activation')
    return True


def create_delivery(node, header):
    from arena_autonomy.race_runtime_policy.evaluator import evaluator_factory, supported
    from arena_autonomy.runtime_dual_state import RuntimeDualProcess
    from arena_autonomy.runtime_dual_delivery import RuntimeDualDelivery
    selected = evaluator_header(node, header)
    worker = RuntimeDualProcess(selected, evaluator_factory=evaluator_factory, supported=supported)
    complete_plan_trace = getattr(node, 'complete_plan_trace', False)
    if complete_plan_trace:
        # 자식에게 전달한 header 원문을 보존한다. 제출/완료 기록은 발행 증명이 아니다.
        node.follow_trace.emit('runtime_evaluator_header', dict(schema=1, header=selected,
            scope='submitted_and_collected_plan_only', publication_proven=False))
    return RuntimeDualDelivery(worker, node.now_s, node.follow_trace.emit,
                               complete_plan_trace=complete_plan_trace)


def control(node, parent_control):
    from arena_autonomy.follow_trace import snapshot_call
    delivery = node.decision_delivery
    notice = node.decision_notice
    if notice is not None and notice.fault:
        delivery.worker.fault = 'completion_notice: '+notice.fault
    node.async_inputs_ready = False
    node.decision_command_stamp_s = None
    node.decision_output_dt_s = max(0., min(.1, node.now_s()-node.last_request_time_s))
    try:
        result = parent_control()
    except BaseException:
        delivery.invalidate()
        raise
    if delivery.pending_publication is not None:
        ack = delivery.acknowledge_publication(published=node.command_was_published,
            reference_s=node.decision_command_stamp_s, speed_mps=node.last_speed,
            steering_rad=node.last_steering, now_s=node.now_s())
        node.follow_trace.emit('route_publication', dict(**ack,
            reference_s=node.decision_command_stamp_s, at_s=node.now_s(),
            speed_mps=node.last_speed, steering_rad=node.last_steering))
    if node.command_was_published:
        node.last_request_time_s = node.now_s()
        delivery.last_application = node.last_request_time_s
    worker = delivery.worker
    if node.async_inputs_ready and not worker.fault and worker.plan.inflight is None and worker.saved_plan is None:
        reference = node.motion.last_meta['reference_stamp_s']
        # 기존 센서 준비 경계를 통과했어도 실제 운동 표본 없이 0을 합성하지 않는다.
        if not node.motion.history.wheels.values:
            return result
        speed = node.motion.history.wheels.values[-1]
        snapshot = snapshot_call(node.scan, node.motion.history, reference, node.motion.counts['resets'],
            speed, node.side, node.last_steering, .1, node.motion.verified, node.rear_blind_half_angle,
            timing_policy=node.motion.timing_policy)
        delivery.offer(snapshot)
    return result

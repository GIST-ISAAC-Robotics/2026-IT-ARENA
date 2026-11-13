"""선택형 두 계산 노드 연결. 기존 발행·신호·독립 보호를 통과한 결과만 확인한다."""
import math

POLICY = 'cad_runtime_r377_v1'


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
    selected = dict(header, runtime_policy=POLICY,
                    runtime_time_policy=node.motion.timing_policy)
    worker = RuntimeDualProcess(selected, evaluator_factory=evaluator_factory, supported=supported)
    return RuntimeDualDelivery(worker, node.now_s, node.follow_trace.emit)


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

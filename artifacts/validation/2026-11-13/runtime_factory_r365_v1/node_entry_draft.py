"""r353 정책 + r362 backend의 명시 시뮬 prefix 초안. 설치/실행 검증 전이다."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import runpy
import sys

from factory_reference import evaluator_factory, policy_support


def install(profile):
    record = json.loads(Path(profile).read_bytes())
    if record != dict(schema='r365_r353_runtime_dual_sim_draft_v1', policy='r353', enabled=True):
        raise ValueError('explicit exact simulation-only draft profile required')
    from arena_autonomy.g1_g2_node import G1G2Node
    from arena_autonomy.local_pursuit import LocalPursuit
    from arena_autonomy.completion_notice import CompletionNotice
    from arena_autonomy.follow_trace import snapshot_call
    from arena_autonomy.runtime_dual_state import RuntimeDualProcess
    from arena_autonomy.runtime_dual_delivery import RuntimeDualDelivery
    original_initialize, original_control = G1G2Node.__init__, G1G2Node.control

    def initialize(node):
        original_initialize(node)
        if (node.get_parameter('use_sim_time').value is not True or node.hardware_shadow
                or not node.recheck_in_worker or not node.decision_delivery.retain_expired_reference
                or node.get_parameter('g1_numeric_execution').value != 'native_v1'):
            raise ValueError('r353 registered native simulation selection required')
        # r336와 같은 선택형 연결. 후속 공통 hook 이관에서는 이 최초 직렬 자식 생성을 생략한다.
        notice_cleanup = node.decision_notice.close()
        if notice_cleanup['alive']:
            raise RuntimeError('prior completion notice did not close')
        old = node.decision_delivery.worker.close()
        if old['forced'] or old['exitcode'] != 0:
            raise RuntimeError('prior serial worker did not close normally')
        header = dict(schema=5, controller_mode='g1_g2_handoff_trial', numeric_execution='native_v1',
            r365_policy='r353', motion_config=asdict(node.motion.history.config),
            follow_config=asdict(node.follow.config), execution_config=asdict(node.follow.execution))
        worker = RuntimeDualProcess(header, evaluator_factory=evaluator_factory, supported=policy_support)
        node.decision_delivery = RuntimeDualDelivery(worker, node.now_s, node.follow_trace.emit)
        node.decision_notice = CompletionNotice(worker.connection, node.decision_guard.trigger)
        node.follow_trace.emit('runtime_factory_policy', dict(policy='r353', backend='runtime_dual_r362',
            profile_sha256=hashlib.sha256(Path(profile).read_bytes()).hexdigest(),
            plan_delay_s=0., recheck_delay_s=0., hardware_enabled=False, default_changed=False))

    def control(node):
        delivery = node.decision_delivery
        if delivery is None:
            return LocalPursuit.control(node)
        notice = node.decision_notice
        if notice is not None and notice.fault:
            delivery.worker.fault = 'completion_notice: '+notice.fault
        node.async_inputs_ready = False
        node.decision_command_stamp_s = None
        node.decision_output_dt_s = max(0., min(.1, node.now_s()-node.last_request_time_s))
        try:
            result = LocalPursuit.control(node)
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
            speed = node.motion.history.wheels.values[-1] if node.motion.history.wheels.values else 0.
            snapshot = snapshot_call(node.scan, node.motion.history, reference, node.motion.counts['resets'],
                speed, node.side, node.last_steering, .1, node.motion.verified, node.rear_blind_half_angle,
                timing_policy=getattr(node.motion, 'timing_policy', 'verified_only'))
            delivery.offer(snapshot)
        return result

    G1G2Node.__init__, G1G2Node.control = initialize, control

    def restore():
        G1G2Node.__init__, G1G2Node.control = original_initialize, original_control
    return restore


def main():
    profile, executable, *arguments = sys.argv[1:]
    if Path(executable).name != 'g1_g2_handoff_trial':
        raise ValueError('unexpected executable')
    install(profile)
    sys.argv = [executable, *arguments]
    runpy.run_path(executable, run_name='__main__')


if __name__ == '__main__':
    main()

"""명시 prefix의 시뮬 제어 노드에만 r334/r336을 연결한다."""
from dataclasses import asdict
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import runpy
import sys

from ros_delivery import Delivery
from ros_worker import Worker

A = Path(__file__).resolve().parent


def install(profile):
    record = json.loads(Path(profile).read_bytes())
    assert record['schema'] == 'r336_published_route_sim_v1'
    delays = (record['plan_delay_s'], record['recheck_delay_s'])
    assert all(type(v) in (int, float) and math.isfinite(v) and 0 <= v <= .2 for v in delays)
    from arena_autonomy.g1_g2_node import G1G2Node
    from arena_autonomy.local_pursuit import LocalPursuit
    from arena_autonomy.completion_notice import CompletionNotice
    from arena_autonomy.follow_trace import snapshot_call
    sys.path.append(str(A.parents[1]/'2026-11-12/current_region_runtime_r282_v1'))
    spec = importlib.util.spec_from_file_location('worker_candidate',
        A.parents[1]/'2026-11-12/current_region_runtime_r282_v1/worker_candidate.py')
    child_policy = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = child_policy
    spec.loader.exec_module(child_policy)
    original_initialize, original_control = G1G2Node.__init__, G1G2Node.control

    def initialize(node):
        original_initialize(node)
        assert node.get_parameter('use_sim_time').value and not node.hardware_shadow
        assert node.recheck_in_worker and node.decision_delivery.retain_expired_reference
        assert node.get_parameter('g1_numeric_execution').value == 'native_v1'
        assert not node.decision_notice.close()['alive']
        old = node.decision_delivery.worker.close()
        assert not old['forced'] and old['exitcode'] == 0
        child_policy.install()
        deadline = [None]

        def wake():
            timer.cancel()
            deadline[0] = None
            node.decision_delivery.collect()
            node.control()

        timer = node.create_timer(3600., wake)
        timer.cancel()

        def arm(due):
            if deadline[0] is not None and deadline[0] <= due:
                return
            deadline[0] = due
            timer.timer_period_ns = max(1000, math.ceil(max(1e-6, due-node.now_s())*1e9))
            timer.reset()

        header = dict(schema=5, controller_mode='g1_g2_handoff_trial', numeric_execution='native_v1',
                      motion_config=asdict(node.motion.history.config),
                      follow_config=asdict(node.follow.config), execution_config=asdict(node.follow.execution))
        worker = Worker(header, clock=node.now_s, delays=delays, wake=arm)
        node.decision_delivery = Delivery(worker, node.now_s, node.follow_trace.emit)
        node.decision_notice = CompletionNotice(worker.connection, node.decision_guard.trigger)
        node.follow_trace.emit('published_route_policy', dict(candidate='r334_r336',
            profile_sha256=hashlib.sha256(Path(profile).read_bytes()).hexdigest(),
            plan_delay_s=delays[0], recheck_delay_s=delays[1],
            submission='latest_unused_native_v1', retained_path_after_publication=True,
            fresh_current_check_required=True, geometry_budget_s=.5, command_budget_s=.2,
            source_budget_s=.3, old_occupancy_permission_reused=False,
            default_changed=False, hardware_enabled=False))

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
            # 발행 여부를 확인할 수 없는 예외에서는 보관 경로를 만들지 않는다.
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
        w = delivery.worker
        if node.async_inputs_ready and not w.fault and w.plan.inflight is None and w.saved_plan is None:
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
        child_policy.overlap._worker = child_policy.ORIGINAL
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

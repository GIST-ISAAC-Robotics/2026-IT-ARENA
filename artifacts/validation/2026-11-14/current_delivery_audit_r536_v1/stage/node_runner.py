"""최신 명시 설치 노드의 r536 scalar 계측 경계. 판단/TTL/센서값은 바꾸지 않는다."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import signal
import time
from observation import ControlReceipt, GuardReceipt, ScalarProbe, ObservedPublisher, hook_controller
from contracts import assert_installed_package

PREFIX = '/replay_r508'
TOPICS = ['/clock', '/scan/native', '/camera/color/image_raw', '/imu/data', '/drive_motor/encoder',
    '/drive', '/drive/safe', '/autonomy/status', '/safety/status', '/autonomy/enable', '/autonomy/reset',
    '/race/command', '/race/command_result', '/race/operator_link', '/race/input_boundary',
    '/actuation/status', '/actuation/clear_stop', '/actuation/arm', '/actuation/stop', '/actuation/reset', '/actuation/disarm',
    '/diagnostics/timing']


def ros_arguments(profile):
    result = ['--ros-args', '-r', '__ns:=' + PREFIX, '--params-file', str(profile)]
    for topic in TOPICS:
        result += ['-r', topic + ':=' + PREFIX + topic]
    return result


def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf8')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('role', choices=('controller', 'guard', 'ops'))
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--installed-root', type=Path, required=True)
    args = p.parse_args()
    out = args.output
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import SetBool, Trigger
    import arena_autonomy
    assert_installed_package(arena_autonomy.__file__, args.installed_root)
    interrupted = False
    def stop(*_):
        nonlocal interrupted
        interrupted = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    rclpy.init(args=ros_arguments(out / 'profile.yaml'), signal_handler_options=SignalHandlerOptions.NO)
    owner = executor = worker = vision = None
    summary = {}
    probe = ScalarProbe(args.role)
    try:
        if args.role == 'controller':
            from arena_autonomy.g1_g2_node import G1G2Node
            from arena_autonomy.runtime_node_backend import evaluator_header
            class ObservedG1(ControlReceipt,G1G2Node):pass

            owner = ObservedG1()
            assert owner.runtime_dual_selected and owner.publisher.topic_name == PREFIX + '/drive'
            worker = owner.decision_delivery.worker
            vision = owner.vision_pipeline.worker.process
            summary['configuration'] = evaluator_header(owner, dict(
                motion_config=asdict(owner.motion.history.config), follow_config=asdict(owner.follow.config),
                execution_config=asdict(owner.follow.execution)))
            summary['child_pids'] = [worker.plan.process.pid, worker.current.process.pid, vision.pid]
        elif args.role == 'guard':
            from arena_autonomy.lidar_safety import LidarSafety
            class ObservedGuard(GuardReceipt,ControlReceipt,LidarSafety):pass
            owner = ObservedGuard()
            assert owner.publisher.topic_name == PREFIX + '/drive/safe'
        else:
            from arena_autonomy.race_ops_node import RaceOps
            owner = RaceOps()
            assert owner.server.server_address[0] == '127.0.0.1'
            summary['http_port'] = owner.server.server_address[1]
        owner._receipt = probe
        if args.role in ('controller','guard'):
            owner.publisher = ObservedPublisher(owner.publisher, owner, args.role)
            subscriptions = owner.motion._ready_motion_subscriptions
            assert len(subscriptions)==2
            assert {s.topic_name for s in subscriptions}=={PREFIX+'/drive_motor/encoder',PREFIX+'/imu/data'}
        if args.role=='controller':
            assert owner.follow_trace.detail=='compact_v1' and not owner.complete_plan_trace
            assert owner.get_parameter('g1_trace_detail').value=='compact_v1'
            hook_controller(owner)
        from replay_transport import audit_current
        audit_current(args.role)
        summary.update(role=args.role, namespace=owner.get_namespace(), node_name=owner.get_name(),
            module_root=str(Path(arena_autonomy.__file__).resolve()), use_sim_time=owner.get_parameter('use_sim_time').value)
        summary['subscriptions']=sorted(subscription.topic_name for subscription in owner.subscriptions)
        if hasattr(owner,'motion'):
            summary['input_contract']=dict(imu_frame=owner.motion.expected_imu_frame,
                lidar_frame=owner.motion.history.config.lidar_frame_id,
                input_representation=owner.get_parameter('lidar_input_representation').value,
                motion_time_policy=owner.motion.timing_policy,motion_verified=owner.motion.verified,
                feedback_mode=owner.motion.feedback_mode,gear_ratio=owner.motion.gear_ratio)
        assert summary['namespace'] == PREFIX and summary['use_sim_time'] is True
        executor = SingleThreadedExecutor()
        executor.add_node(owner)
        if args.role=='controller':
            summary['trace_detail']=owner.follow_trace.detail
        write(out / (args.role + '_ready.json'), summary)
        started = time.monotonic()
        motion_ready_written=False
        while not interrupted and time.monotonic() - started < 58:
            executor.spin_once(timeout_sec=.01)
            if not motion_ready_written and hasattr(owner,'motion') and owner.motion.counts['gyro']>0 and owner.motion.counts['drive_motor']>0:
                ready_path=out/(args.role+'_motion_accepted.json')
                if not ready_path.exists():
                    write(ready_path,dict(counts=dict(owner.motion.counts),
                        clock_s=owner.get_clock().now().nanoseconds*1e-9,input_contract=summary['input_contract']))
                    motion_ready_written=True
            if worker and worker.fault:
                raise RuntimeError('runtime worker fault: ' + worker.fault)
        if hasattr(owner, 'on_enable'):
            owner.on_enable(SetBool.Request(data=False), SetBool.Response())
        if worker:
            summary['trace_detail'] = owner.follow_trace.detail
            summary['delivery_counts'] = dict(owner.decision_delivery.counts)
            summary['worker_fault'] = worker.fault
            summary['vision'] = owner.vision_pipeline.snapshot()
        if hasattr(owner, 'motion'):
            summary['motion_counts'] = dict(owner.motion.counts)
            summary['motion_meta'] = owner.motion.last_meta
        if hasattr(owner,'timing'):
            response=owner.timing.export(Trigger.Request(),Trigger.Response())
            summary['timing_export']=dict(success=response.success,message=response.message,
                completed=dict(owner.timing.completed),dropped=owner.timing.dropped)
            assert response.success,response.message
    except BaseException as error:
        summary['error'] = type(error).__name__ + ': ' + str(error)
        raise
    finally:
        if executor:
            executor.shutdown(timeout_sec=1.)
        if owner:
            owner.destroy_node()
        if worker:
            summary['cleanup'] = worker.cleanup
            summary['vision_exitcode'] = vision.exitcode
            summary['vision_alive'] = vision.is_alive()
        rclpy.shutdown()
        write(out / (args.role + '_summary.json'), summary)
        write(out/(args.role+'_scalar_summary.json'), probe.write(out))


if __name__ == '__main__':
    main()

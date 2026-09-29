#!/usr/bin/env python3
"""A2 격리 재생: 센서만 /replay 아래로 발행하고 구동 출력은 비교만 한다."""
import argparse
from collections import Counter, defaultdict
import json
import math
import os
from pathlib import Path
import platform
import random
import signal
import subprocess
import sys
import time
import zipfile

from audit_sensor_bag import audit, reader_for, summarize_timing, summarize_stages, REPO
from replay_observer import ReplayObserver, sequence_coverage
from replay_timing import load_export
from replay_imu_probe import ImuProbe, NODE_NAME as IMU_PROBE_NAME
from replay_transport import MODES, configure as configure_transport, audit_current
from arena_vehicle_interface.bag_contract import INPUTS, TOPICS, replay_topic, quantiles, sha256_file, eof_stopped, compare_commands, active_window_continuity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--timing-mode', choices=('topic', 'buffered'), default='topic',
                        help='기존 콜백별 진단 발행 또는 EOF까지 메모리 보존 후 저장 비교')
    parser.add_argument('--executor-mode', choices=('legacy', 'retained'),
                        help='명시적 실행 비교. 생략 시 기록 매개변수/제어기 기본값 유지')
    parser.add_argument('--trace-wheels-publication', action='store_true',
                        help='엔코더 발행 시작/끝과 콜백 진입을 같은 호스트에서 대조하는 선택 계측')
    parser.add_argument('--trace-imu-publication', action='store_true',
                        help='IMU 원본 시각과 발행 시작/끝을 보존하는 선택 계측; 기본 비활성')
    parser.add_argument('--imu-probe', action='store_true',
                        help='같은 QoS의 최소 IMU 수신 프로세스 추가; 기본 비활성, live DDS 감사 필수')
    parser.add_argument('--dds-transport', choices=MODES, default='system')
    parser.add_argument('--dds-audit-library', type=Path,
                        help='Humble/Fast DDS용 읽기 전용 participant 검사 공유 라이브러리')
    args = parser.parse_args()
    if args.imu_probe and args.dds_audit_library is None:
        parser.error('--imu-probe requires --dds-audit-library')
    # Transport options are replay-only and audited before sensor publication.
    if not math.isfinite(args.rate) or not .1 <= args.rate <= 2.:
        raise ValueError('Replay rate must be within .1..2')
    root, output = args.recording.resolve(), args.output.resolve()
    if not output.is_relative_to(REPO/'artifacts'):
        raise ValueError('Output must be under artifacts')
    output.mkdir(parents=True, exist_ok=False)
    sources = [Path(__file__).resolve(), REPO/'scripts/audit_sensor_bag.py', REPO/'scripts/replay_observer.py',
        REPO/'scripts/replay_transport.py', REPO/'scripts/dds_transport_probe.cpp',
        REPO/'scripts/replay_imu_probe.py', REPO/'scripts/replay_timing.py',
        *sorted((REPO/'src/arena_autonomy/arena_autonomy').glob('*.py')),
        REPO/'src/arena_vehicle_interface/arena_vehicle_interface/timing_probe.py',
        REPO/'src/arena_vehicle_interface/arena_vehicle_interface/queued_log.py',
        REPO/'src/arena_vehicle_interface/arena_vehicle_interface/node_lifecycle.py',
        REPO/'src/arena_vehicle_interface/arena_vehicle_interface/bag_contract.py']
    source_hashes = {str(p.relative_to(REPO)): sha256_file(p) for p in sources}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as snapshot:
        for p in sources:
            snapshot.write(p, str(p.relative_to(REPO)))
    # 검증 실패 자료는 어떤 노드도 시작하거나 발행하기 전에 거부한다.
    original = audit(root)
    (output/'original_audit.json').write_text(json.dumps(original, indent=2))
    parameters = json.loads((root/'parameters.json').read_text())
    if set(parameters) != {'local_pursuit', 'lidar_safety'}:
        raise ValueError('Both controller parameter snapshots required')
    for values in parameters.values():
        if values.get('lidar_compensation') != 'both' or values.get('motion_imu_topic') != '/imu/data':
            raise ValueError('Unexpected motion input contract')
    # 자율주행/보호층 외에는 시작하지 않는다. 실행 중인 차량 도메인도 상속하지 않는다.
    domain = random.SystemRandom().randrange(120, 160)
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    # 외부 discovery 서버/정적 피어 설정을 재생 세션에 가져오지 않는다.
    for key in ('ROS_STATIC_PEERS', 'ROS_DISCOVERY_SERVER'):
        os.environ.pop(key, None)
    transport_config = configure_transport(args.dds_transport, args.dds_audit_library, output)
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosidl_runtime_py.utilities import get_message
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from ackermann_msgs.msg import AckermannDriveStamped
    import yaml

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('sensor_replay', namespace='/replay')
    try:
        audit_current('sensor_replay')
    except BaseException:
        node.destroy_node()
        rclpy.shutdown()
        raise
    observer = ReplayObserver(output/'observer')
    imu_probe = ImuProbe(output/'imu_probe') if args.imu_probe else None
    imu_probe_result = None
    processes, logs, errors = [], [], []
    timing, commands, statuses = [], defaultdict(list), defaultdict(list)
    completed_counts = {}
    timing_exports = {}
    buffered_timing = []
    text_log_stats = {}
    vision_stats = {}
    published = Counter()
    late_ms = []
    wheel_publications = []
    imu_publications = []
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    simulation_ns = original['first_ns']
    last_clock_ns = -1
    tail_ns = original['last_ns']
    types = {topic: get_message(name) for topic, name in INPUTS.items()}
    publishers = {topic: node.create_publisher(types[topic], replay_topic(topic),
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)) for topic in INPUTS}
    clock = node.create_publisher(Clock, '/replay/clock', 10)
    def advance(ns):
        nonlocal simulation_ns, last_clock_ns
        # 재생기가 늦더라도 시계를 과거 메시지의 시각으로 되감지 않는다.
        simulation_ns = max(simulation_ns, ns)
        if simulation_ns == last_clock_ns:
            return
        last_clock_ns = simulation_ns
        message = Clock()
        message.clock.sec, message.clock.nanosec = divmod(simulation_ns, 1_000_000_000)
        clock.publish(message)
    def spin_for(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline and not stopping:
            observer.check()
            if imu_probe:
                imu_probe.check()
            rclpy.spin_once(node, timeout_sec=min(.005, max(0., deadline-time.monotonic())))
    endpoint_audit = {}
    ready_snapshots = {}
    discovery_retries = []
    began = time.monotonic()
    try:
        observer.start()
        spin_for(1.)
        others = [n for n in node.get_node_names() if n not in ('sensor_replay', 'sensor_replay_observer')]
        if others:
            raise RuntimeError('Selected replay domain is occupied: '+str(others))
        if imu_probe:
            imu_probe.start()
        remaps = {t: '/replay'+t for t in TOPICS}
        remaps.update({'/clock': '/replay/clock', '/autonomy/enable': '/replay/autonomy/enable',
                       '/autonomy/reset': '/replay/autonomy/reset'})
        for name, values in parameters.items():
            params = dict(values, use_sim_time=True, timing_probe=True,
                          timing_probe_mode=args.timing_mode, timing_probe_capacity=100000,
                          timing_probe_buffer_path=str(output/(name+'_timing_buffer.jsonl'))
                          if args.timing_mode == 'buffered' else '')
            if args.executor_mode is not None:
                params['executor_mode'] = args.executor_mode
            path = output/(name+'_parameters.yaml')
            path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
            log = (output/(name+'.log')).open('w')
            logs.append(log)
            command = [sys.executable, '-c', f'from arena_autonomy.{name} import main; main()',
                       '--ros-args', '--params-file', str(path), '-r', '__ns:=/replay']
            if args.dds_audit_library is not None:
                command[2] = (f'import sys; sys.path.insert(0, {str(REPO/"scripts")!r}); '
                              f'from replay_transport import run_controller; run_controller({name!r})')
            for old, new in remaps.items():
                command += ['-r', old+':='+new]
            processes.append(subprocess.Popen(command, stdout=log, stderr=log, stdin=subprocess.DEVNULL,
                                               start_new_session=True))
        deadline = time.monotonic()+20
        required_subscribers = {'/scan': 2, '/imu/data': 3 if imu_probe else 2, '/wheel_states': 2}
        while not all(node.count_subscribers(replay_topic(t)) >= n for t, n in required_subscribers.items()) or node.count_subscribers('/replay/camera/color/image_raw') < 1:
            if any(p.poll() is not None for p in processes) or time.monotonic() > deadline:
                raise RuntimeError('Replay controllers failed to connect')
            spin_for(.05)
        # DDS endpoint 수와 노드 이름 조회 갱신은 동시에 완료되지 않을 수 있다.
        # 센서는 아직 보내지 않고 실제 노드별 경로까지 확인한 뒤 시작한다.
        while len(endpoint_audit) < 2:
            for name in parameters:
                try:
                    subscriptions = node.get_subscriber_names_and_types_by_node(name, '/replay')
                    publications = node.get_publisher_names_and_types_by_node(name, '/replay')
                except Exception as error:
                    if type(error).__name__ != 'NodeNameNonExistentError':
                        raise
                    discovery_retries.append({'node': name, 'error': str(error)})
                    continue
                inputs = {t for t, _ in subscriptions}
                outputs = {t for t, _ in publications}
                required_inputs = {'/replay/scan', '/replay/imu/data', '/replay/wheel_states', '/replay/clock'}
                required_inputs.add('/replay/drive' if name == 'lidar_safety' else '/replay/camera/color/image_raw')
                required_outputs = {'/replay/diagnostics/timing', '/replay/drive/safe' if name == 'lidar_safety' else '/replay/drive'}
                if not required_inputs <= inputs or not required_outputs <= outputs:
                    continue
                if any(not topic.startswith('/replay/') and topic not in ('/rosout', '/parameter_events')
                       for topic in inputs | outputs):
                    raise RuntimeError('Non-isolated controller endpoint')
                endpoint_audit[name] = dict(subscriptions=subscriptions, publications=publications)
            if len(endpoint_audit) == 2:
                break
            if any(p.poll() is not None for p in processes) or time.monotonic() > deadline or stopping:
                raise RuntimeError('Replay node discovery incomplete before deadline')
            spin_for(.05)
        if any(t in dict(node.get_topic_names_and_types()) for t in ('/drive', '/drive/safe', '/cmd_vel', '/actuation/tx')):
            raise RuntimeError('Actuation topic unexpectedly present')
        # DDS 그래프 등록은 생성자가 끝나고 executor가 응답한다는 뜻이 아니다.
        # 센서 공급 전 서비스 응답으로 실제 초기화 완료를 확인한다.
        for name in parameters:
            client = node.create_client(Trigger, '/replay/'+name+'/timing_snapshot')
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('Controller readiness service missing: '+name)
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic()+5.
            while not future.done() and time.monotonic() < deadline and not stopping:
                spin_for(.01)
            if not future.done() or not future.result().success:
                raise RuntimeError('Controller did not become ready: '+name)
            ready_snapshots[name] = json.loads(future.result().message)
            if ready_snapshots[name]['node'] != name:
                raise RuntimeError('Readiness snapshot node mismatch')
            node.destroy_client(client)
        observer.wait_for_publishers(node, {'/replay/diagnostics/timing': 2,
            '/replay/drive': 1, '/replay/drive/safe': 1,
            '/replay/autonomy/status': 1, '/replay/safety/status': 1})
        if args.dds_audit_library is not None:
            audited_names = ['sensor_replay', 'sensor_replay_observer', *parameters]
            if imu_probe:
                audited_names.append(IMU_PROBE_NAME)
            for name in audited_names:
                if not (output/f'dds_active_{name}.json').is_file():
                    raise RuntimeError('Missing live transport audit: '+name)
        advance(simulation_ns)
        spin_for(.2)
        reader = reader_for(root)
        start_wall = time.monotonic()
        first_ns = original['first_ns']
        last_clock_wall = 0.
        while reader.has_next() and not stopping:
            observer.check()
            if imu_probe:
                imu_probe.check()
            topic, payload, timestamp = reader.read_next()
            # 비교용 출력/진단은 사전 감사에서 이미 읽었다. 여기서 시계/스핀을
            # 다시 실행하면 느린 시뮬레이션의 wall watchdog 상태가 큰 부하가 된다.
            if topic not in INPUTS:
                continue
            trace_rows = (wheel_publications if args.trace_wheels_publication and topic == '/wheel_states'
                          else imu_publications if args.trace_imu_publication and topic == '/imu/data' else None)
            trace_source_ns = None
            if trace_rows is not None:
                from rclpy.serialization import deserialize_message
                motion_message = deserialize_message(payload, types[topic])
                trace_source_ns = motion_message.header.stamp.sec*1_000_000_000+motion_message.header.stamp.nanosec
            target = start_wall+(timestamp-first_ns)/1e9/args.rate
            while time.monotonic() < target and not stopping:
                now_wall = time.monotonic()
                if now_wall-last_clock_wall >= .005:
                    advance(first_ns+int((now_wall-start_wall)*args.rate*1e9))
                    last_clock_wall = now_wall
                # 이 구간에는 서비스 응답/수신 콜백이 없다. 관측은 별도 프로세스가
                # 담당하므로 메시지마다 executor를 붙였다 떼며 기다리지 않는다.
                time.sleep(min(.001, max(0., target-time.monotonic())))
            if stopping:
                break
            # rosbag의 CDR를 rclpy의 직렬화 메시지 경로로 그대로 보낸다.
            # 특히 RGB를 Python 객체로 풀었다가 다시 직렬화하지 않는다.
            now_wall = time.monotonic()
            advance(first_ns+int((now_wall-start_wall)*args.rate*1e9))
            publish_began = time.perf_counter_ns() if trace_rows is not None else None
            publishers[topic].publish(payload)
            publish_ended = time.perf_counter_ns() if trace_rows is not None else None
            late_ms.append(max(0., (time.monotonic()-target)*1000))
            if trace_rows is not None:
                trace_rows.append(dict(source_ns=trace_source_ns, recorded_receive_ns=timestamp,
                    replay_ros_ns=simulation_ns, begin_monotonic_ns=publish_began,
                    end_monotonic_ns=publish_ended, schedule_lateness_wall_ms=late_ms[-1]))
            published[topic] += 1
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('Controller exited during replay')
        # 입력만 끊고 시계는 더 진행: 오래된 관측으로 계속 주행하지 않는지 확인한다.
        tail_start = time.monotonic()
        tail_ns = max(simulation_ns, original['last_ns'])
        while time.monotonic()-tail_start < 1.5/args.rate and not stopping:
            advance(tail_ns+int((time.monotonic()-tail_start)*args.rate*1e9))
            spin_for(.005)
        # EOF 상태를 유지하며 콜백 완료 횟수를 제어기에서 직접 읽는다.
        # 진단 토픽을 수신한 개수만으로 센서 손실을 단정하지 않는다.
        spin_for(.2)
        for name in parameters:
            service = 'timing_export' if args.timing_mode == 'buffered' else 'timing_snapshot'
            client = node.create_client(Trigger, '/replay/'+name+'/'+service)
            if not client.wait_for_service(timeout_sec=3.):
                raise RuntimeError('Timing snapshot service missing: '+name)
            future = client.call_async(Trigger.Request())
            # 센서 공급/EOF 검사는 이미 끝났다. 파일 저장 응답 여유이며 주행 한도 아님.
            deadline = time.monotonic()+(15. if args.timing_mode == 'buffered' else 3.)
            while not future.done() and time.monotonic() < deadline and not stopping:
                spin_for(.01)
            if not future.done() or not future.result().success:
                raise RuntimeError('Timing snapshot failed: '+name)
            snapshot = json.loads(future.result().message)
            if snapshot['node'] != name:
                raise RuntimeError('Timing snapshot node mismatch')
            if args.timing_mode == 'buffered':
                buffered_timing.extend(load_export(snapshot, output/(name+'_timing_buffer.jsonl'), name))
                timing_exports[name] = snapshot['timing_export']
            completed_counts.update({name+'/'+key: value for key, value in snapshot['completed'].items()})
            if 'text_log' in snapshot:
                text_log_stats[name] = snapshot['text_log']
            if 'vision' in snapshot:
                vision_stats[name] = snapshot['vision']
            node.destroy_client(client)
        if stopping:
            errors.append('interrupted')
    except Exception as error:
        errors.append(repr(error))
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
                errors.append('forced controller cleanup')
        for log in logs:
            log.close()
        if imu_probe:
            try:
                imu_probe_result = imu_probe.close()
            except Exception as error:
                errors.append(repr(error))
        try:
            observer.close()
        except Exception as error:
            errors.append(repr(error))
        try:
            collected = observer.snapshot()
        except Exception as error:
            errors.append(repr(error))
            collected = {}
        timing = collected.get('timing', [])
        if args.timing_mode == 'buffered':
            if timing:
                errors.append('Unexpected per-callback timing publication in buffered mode')
            timing = sorted(buffered_timing, key=lambda row: row['begin_monotonic_ns'])
        for topic in ('/drive', '/drive/safe'):
            commands[topic] = collected.get(topic, [])
        for topic in ('/autonomy/status', '/safety/status'):
            statuses[topic] = collected.get(topic, [])
        node.destroy_node()
        rclpy.shutdown()
    stopped = eof_stopped(commands['/drive/safe'], tail_ns)
    if source_hashes != {str(p.relative_to(REPO)): sha256_file(p) for p in sources}:
        errors.append('source changed during replay')
    source_counts = {t: original['counts'][t] for t in INPUTS}
    sample_counts = Counter(r['node']+'/'+r['callback'] for r in timing)
    callback_topics = {'scan': '/scan', 'imu': '/imu/data', 'wheels': '/wheel_states',
                       'image': '/camera/color/image_raw'}
    delivery = {name+'/'+callback: dict(published=source_counts[topic],
        callbacks=completed_counts.get(name+'/'+callback),
        diagnostic_rows=sample_counts[name+'/'+callback],
        ratio=completed_counts.get(name+'/'+callback, 0)/source_counts[topic])
        for name in parameters for callback, topic in callback_topics.items()
        if callback != 'image' or name == 'local_pursuit'}
    functional = (not errors and dict(published) == source_counts and stopped and bool(timing)
                  and any(abs(r[1]) > .1 for r in commands['/drive/safe'])
                  and len(processes) == 2 and all(p.returncode == 0 for p in processes))
    lateness_sim = quantiles([value*args.rate for value in late_ms])
    # 시험 전달기의 기준: p99는 제어 1주기(20ms), 최대는 스캔 1주기(100ms) 이내.
    # 제품 실시간 안전 인증이 아니며 기능/입력 완전 전달과 따로 보고한다.
    schedule_ok = bool(late_ms and lateness_sim['p99'] <= 20. and lateness_sim['max'] <= 100.)
    coverage = sequence_coverage(timing, completed_counts)
    measurement_ok = bool(coverage and all(v['complete'] for v in coverage.values()))
    continuity = active_window_continuity(original['commands']['/drive/safe'], commands['/drive/safe'])
    report = dict(passed=bool(functional and schedule_ok), functional_passed=bool(functional),
        active_window_continuity=continuity,
        operational_passed=bool(functional and schedule_ok and measurement_ok and continuity['no_interruption']),
        executor_mode_override=args.executor_mode,
        timing_mode=args.timing_mode, timing_exports=timing_exports,
        dds_transport=transport_config,
        wheel_publication_trace_count=len(wheel_publications),
        imu_publication_trace_count=len(imu_publications),
        imu_probe_enabled=args.imu_probe, imu_probe_result=imu_probe_result,
        diagnostic_sequence_coverage=coverage, measurement_passed=measurement_ok,
        schedule_passed=schedule_ok, schedule_lateness_sim_ms=lateness_sim,
        schedule_limits_sim_ms=dict(p99=20., maximum=100.),
        error=errors, domain=domain, rate=args.rate, wall_duration_s=time.monotonic()-began,
        text_log_stats=text_log_stats,
        vision_stats=vision_stats,
        ready_snapshots=ready_snapshots,
        runtime={'platform': platform.platform(), 'machine': platform.machine(),
                 'python': platform.python_version(), 'ros_distro': os.environ.get('ROS_DISTRO')},
        payload_mode='unchanged_serialized_cdr',
        completed_callback_counts=completed_counts,
        callback_count_basis=('controller timing_export checkpoint with verified file hash/sequence'
                              if args.timing_mode == 'buffered' else
                              'controller timing_snapshot service; diagnostic topic row count is separate'),
        original_counts=source_counts, published_counts=dict(published), callback_counts=dict(sample_counts),
        input_callback_delivery=delivery,
        input_callback_delivery_complete=all(v['callbacks'] == v['published'] for v in delivery.values()),
        timing=summarize_timing(timing), stages=summarize_stages(timing),
        schedule_lateness_wall_ms=quantiles(late_ms),
        eof_zero_command=stopped, exit_codes=[p.returncode for p in processes], endpoints=endpoint_audit,
        discovery_retries=discovery_retries,
        commands=commands, states={t: dict(Counter(str(r.get('state', r.get('reason', 'unknown'))) for r in rows))
                                  for t, rows in statuses.items()},
        status_records=statuses,
        command_comparison={topic: compare_commands(original['commands'][topic], commands[topic], original['last_ns'])
                            for topic in ('/drive', '/drive/safe')},
        source_sha256=source_hashes,
        scope='open-loop sensor replay; no actuator, dynamics, track safety or Jetson performance certification')
    (output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    with (output/'timing.jsonl').open('w', encoding='utf-8') as stream:
        for row in timing:
            stream.write(json.dumps(row)+'\n')
    if args.trace_wheels_publication:
        with (output/'wheel_publication.jsonl').open('x', encoding='utf-8') as stream:
            for row in wheel_publications:
                stream.write(json.dumps(row)+'\n')
    if args.trace_imu_publication:
        with (output/'imu_publication.jsonl').open('x', encoding='utf-8') as stream:
            for row in imu_publications:
                stream.write(json.dumps(row)+'\n')
    print(json.dumps({k: report[k] for k in ('passed', 'error', 'eof_zero_command', 'exit_codes', 'callback_counts')}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

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

from audit_sensor_bag import audit, reader_for, summarize_timing, REPO
from arena_vehicle_interface.bag_contract import INPUTS, TOPICS, replay_topic, quantiles, sha256_file, eof_stopped, compare_commands


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rate', type=float, default=1.)
    args = parser.parse_args()
    if not math.isfinite(args.rate) or not .1 <= args.rate <= 2.:
        raise ValueError('Replay rate must be within .1..2')
    root, output = args.recording.resolve(), args.output.resolve()
    if not output.is_relative_to(REPO/'artifacts'):
        raise ValueError('Output must be under artifacts')
    output.mkdir(parents=True, exist_ok=False)
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
    processes, logs, errors = [], [], []
    timing, commands, statuses = [], defaultdict(list), defaultdict(list)
    completed_counts = {}
    published = Counter()
    late_ms = []
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    simulation_ns = original['first_ns']
    last_clock_ns = -1
    tail_ns = original['last_ns']
    def drive_callback(topic):
        def callback(msg):
            stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            commands[topic].append([stamp, msg.drive.speed, msg.drive.steering_angle])
        return callback
    for topic in ('/drive', '/drive/safe'):
        node.create_subscription(AckermannDriveStamped, '/replay'+topic, drive_callback(topic), 100)
    node.create_subscription(String, '/replay/diagnostics/timing', lambda m: timing.append(json.loads(m.data)), 1000)
    def status_callback(topic):
        # rclpy는 인자 두 개짜리 콜백에 MessageInfo를 전달한다. 기본값 캡처 대신 factory 사용.
        def callback(message):
            statuses[topic].append(json.loads(message.data))
        return callback
    for topic in ('/autonomy/status', '/safety/status'):
        node.create_subscription(String, '/replay'+topic, status_callback(topic), 100)
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
            rclpy.spin_once(node, timeout_sec=min(.005, max(0., deadline-time.monotonic())))
    endpoint_audit = {}
    discovery_retries = []
    began = time.monotonic()
    try:
        spin_for(1.)
        others = [n for n in node.get_node_names() if n != 'sensor_replay']
        if others:
            raise RuntimeError('Selected replay domain is occupied: '+str(others))
        remaps = {t: '/replay'+t for t in TOPICS}
        remaps.update({'/clock': '/replay/clock', '/autonomy/enable': '/replay/autonomy/enable',
                       '/autonomy/reset': '/replay/autonomy/reset'})
        for name, values in parameters.items():
            params = dict(values, use_sim_time=True, timing_probe=True)
            path = output/(name+'_parameters.yaml')
            path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
            log = (output/(name+'.log')).open('w')
            logs.append(log)
            command = [sys.executable, '-c', f'from arena_autonomy.{name} import main; main()',
                       '--ros-args', '--params-file', str(path), '-r', '__ns:=/replay']
            for old, new in remaps.items():
                command += ['-r', old+':='+new]
            processes.append(subprocess.Popen(command, stdout=log, stderr=log, stdin=subprocess.DEVNULL,
                                               start_new_session=True))
        deadline = time.monotonic()+20
        while not all(node.count_subscribers(replay_topic(t)) >= 2 for t in ('/scan', '/imu/data', '/wheel_states')) or node.count_subscribers('/replay/camera/color/image_raw') < 1:
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
        advance(simulation_ns)
        spin_for(.2)
        reader = reader_for(root)
        start_wall = time.monotonic()
        first_ns = original['first_ns']
        last_clock_wall = 0.
        while reader.has_next() and not stopping:
            topic, payload, timestamp = reader.read_next()
            # 비교용 출력/진단은 사전 감사에서 이미 읽었다. 여기서 시계/스핀을
            # 다시 실행하면 느린 시뮬레이션의 wall watchdog 상태가 큰 부하가 된다.
            if topic not in INPUTS:
                continue
            target = start_wall+(timestamp-first_ns)/1e9/args.rate
            while time.monotonic() < target and not stopping:
                now_wall = time.monotonic()
                if now_wall-last_clock_wall >= .005:
                    advance(first_ns+int((now_wall-start_wall)*args.rate*1e9))
                    last_clock_wall = now_wall
                rclpy.spin_once(node, timeout_sec=min(.002, max(0., target-time.monotonic())))
            if stopping:
                break
            # rosbag의 CDR를 rclpy의 직렬화 메시지 경로로 그대로 보낸다.
            # 특히 RGB를 Python 객체로 풀었다가 다시 직렬화하지 않는다.
            now_wall = time.monotonic()
            advance(first_ns+int((now_wall-start_wall)*args.rate*1e9))
            publishers[topic].publish(payload)
            late_ms.append(max(0., (time.monotonic()-target)*1000))
            published[topic] += 1
            rclpy.spin_once(node, timeout_sec=0.)
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
            client = node.create_client(Trigger, '/replay/'+name+'/timing_snapshot')
            if not client.wait_for_service(timeout_sec=3.):
                raise RuntimeError('Timing snapshot service missing: '+name)
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic()+3.
            while not future.done() and time.monotonic() < deadline and not stopping:
                spin_for(.01)
            if not future.done() or not future.result().success:
                raise RuntimeError('Timing snapshot failed: '+name)
            snapshot = json.loads(future.result().message)
            if snapshot['node'] != name:
                raise RuntimeError('Timing snapshot node mismatch')
            completed_counts.update({name+'/'+key: value for key, value in snapshot['completed'].items()})
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
        node.destroy_node()
        rclpy.shutdown()
    stopped = eof_stopped(commands['/drive/safe'], tail_ns)
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
    report = dict(passed=bool(functional and schedule_ok), functional_passed=bool(functional),
        schedule_passed=schedule_ok, schedule_lateness_sim_ms=lateness_sim,
        schedule_limits_sim_ms=dict(p99=20., maximum=100.),
        error=errors, domain=domain, rate=args.rate, wall_duration_s=time.monotonic()-began,
        runtime={'platform': platform.platform(), 'machine': platform.machine(),
                 'python': platform.python_version(), 'ros_distro': os.environ.get('ROS_DISTRO')},
        payload_mode='unchanged_serialized_cdr',
        completed_callback_counts=completed_counts,
        callback_count_basis='controller timing_snapshot service; diagnostic topic row count is separate',
        original_counts=source_counts, published_counts=dict(published), callback_counts=dict(sample_counts),
        input_callback_delivery=delivery,
        input_callback_delivery_complete=all(v['callbacks'] == v['published'] for v in delivery.values()),
        timing=summarize_timing(timing), schedule_lateness_wall_ms=quantiles(late_ms),
        eof_zero_command=stopped, exit_codes=[p.returncode for p in processes], endpoints=endpoint_audit,
        discovery_retries=discovery_retries,
        commands=commands, states={t: dict(Counter(str(r.get('state', r.get('reason', 'unknown'))) for r in rows))
                                  for t, rows in statuses.items()},
        command_comparison={topic: compare_commands(original['commands'][topic], commands[topic], original['last_ns'])
                            for topic in ('/drive', '/drive/safe')},
        source_sha256={str(p.relative_to(REPO)): sha256_file(p) for p in [Path(__file__).resolve(),
            REPO/'scripts/audit_sensor_bag.py',
            *sorted((REPO/'src/arena_autonomy/arena_autonomy').glob('*.py')),
            REPO/'src/arena_vehicle_interface/arena_vehicle_interface/timing_probe.py',
            REPO/'src/arena_vehicle_interface/arena_vehicle_interface/bag_contract.py']},
        scope='open-loop sensor replay; no actuator, dynamics, track safety or Jetson performance certification')
    (output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    with (output/'timing.jsonl').open('w', encoding='utf-8') as stream:
        for row in timing:
            stream.write(json.dumps(row)+'\n')
    print(json.dumps({k: report[k] for k in ('passed', 'error', 'eof_zero_command', 'exit_codes', 'callback_counts')}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

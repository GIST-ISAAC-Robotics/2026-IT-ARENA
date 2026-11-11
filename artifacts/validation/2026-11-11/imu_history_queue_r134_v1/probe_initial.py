"""동결 r128 현재 명령 계산과 200 Hz ROS 수신 경합. 차량 토픽/센서 접근 없음."""
import argparse
import csv
import hashlib
import json
import multiprocessing as mp
import os
from pathlib import Path
import signal
import sys
import time

TOPIC = '/arena_probe_r134/imu'


def publisher(stop, ready, output):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    rclpy.init(args=[])
    node = Node('r134_publisher')
    pub = node.create_publisher(Imu, TOPIC, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 15
        while not pub.get_subscription_count():
            if stop.is_set() or time.monotonic() > deadline:
                raise RuntimeError('diagnostic subscriber unavailable')
            time.sleep(.01)
        ready.send(True)
        seq = 0
        due = time.monotonic()
        with Path(output).open('x', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['seq', 'stamp_ns', 'wall_s'])
            while not stop.is_set():
                now = time.monotonic()
                if now < due:
                    time.sleep(min(.005, due-now))
                    continue
                due = now + .005
                msg = Imu()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'diagnostic_only'
                msg.angular_velocity.x = float(seq)
                pub.publish(msg)
                writer.writerow([seq, msg.header.stamp.sec*10**9+msg.header.stamp.nanosec, now])
                seq += 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ready.close()


def stats(values):
    import numpy as np
    if not values:
        return {'count': 0}
    return dict(count=len(values), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), maximum=max(values),
                over_20ms=sum(v > 20 for v in values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=['idle', 'rebase'], required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '199':
        raise ValueError('localhost diagnostic domain 199 required')
    root = args.root.resolve()
    sys.path[:0] = [str(root/'src/arena_autonomy'), str(root/'src/arena_vehicle_interface'),
                   str(root/'artifacts/validation/2026-11-11/follow_rebase_r126_v1')]
    from arena_autonomy.decision_process import make_controller
    from arena_autonomy.follow_trace import restore_call, snapshot_call, freeze, equal_output
    from arena_autonomy.scene_motion import motion_scene
    from arena_autonomy.lidar_motion import MotionUnavailable
    from rebase import rebase
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = json.loads((root/'artifacts/validation/2026-11-11/current_rebase_cost_r128_v1/bundle_manifest.json').read_bytes())
    unchanged = lambda: all(sha(root/n) == v['sha256'] for n, v in manifest['files'].items())
    assert unchanged()
    rows = [json.loads(line) for line in (root/manifest['trace']).open(encoding='utf8')]
    owner, motion = make_controller(rows[0]['data'])
    calls = []
    for row in rows:
        if row['kind'] == 'call' and 'current_rebase' in row['data']:
            saved = row['data']['current_rebase']
            calls.append((row, restore_call(saved['input'], motion)))
    assert len(calls) == 316
    # 복원은 실행 전 준비한다. 실제 실행 비용은 snapshot/projection/rebase/freeze다.
    args.output.mkdir(parents=True, exist_ok=False)
    context = mp.get_context('spawn')
    stop = context.Event()
    parent, child = context.Pipe()
    process = context.Process(target=publisher, args=(stop, child, str(args.output/'sent.csv')))
    received, latency, compute, ticks, outputs, mismatches = [], [], [], [], [], []
    state = dict(started=False, index=0)
    fault = None
    rclpy.init(args=[])
    node = Node('r134_receiver')

    def imu(msg):
        now = node.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        received.append((int(msg.angular_velocity.x), stamp, now))
        latency.append((now-stamp)/1e6)

    def timer():
        if state['started']:
            ticks.append(time.monotonic())

    def evaluate():
        if not state['started'] or state['index'] >= len(calls):
            return
        row, (data, scan, history) = calls[state['index']]
        original = row['data']
        if args.mode == 'rebase':
            began = time.perf_counter()
            current = snapshot_call(scan, history, data['reference_s'], data['epoch'], data['speed_mps'],
                data['side'], data['previous_steering_rad'], data['dt_s'],
                data['timing_verified'], data['rear_half_angle_deg'])
            try:
                scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                    epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
                actual = rebase(original['output'], original['input'], current, scene, history, owner.config)
            except (MotionUnavailable, ValueError) as error:
                actual = dict(speed_mps=0., steering_rad=data['previous_steering_rad'],
                    reference_s=data['reference_s'], reason='rebase_input_stop', input_error=str(error),
                    independent_guard_required=True, runtime_authorized=False)
            actual = freeze(actual)
            compute.append((time.perf_counter()-began)*1000)
            outputs.append((row['seq'], actual))
            # 동등성 비교는 실행 종료 뒤 수행해 수신 경합 비용에 섞지 않는다.
        state['index'] += 1

    from copy import deepcopy
    history_qos = deepcopy(qos_profile_sensor_data)
    history_qos.depth = 64
    node.create_subscription(Imu, TOPIC, imu, history_qos)
    node.create_timer(.02, timer)
    node.create_timer(.1, evaluate)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    began = time.monotonic()
    process.start()
    child.close()
    try:
        while not parent.poll():
            executor.spin_once(timeout_sec=.01)
            if time.monotonic()-began > 20:
                raise RuntimeError('publisher readiness timeout')
        assert parent.recv() is True
        state['started'] = True
        while state['index'] < len(calls):
            executor.spin_once(timeout_sec=.01)
            if time.monotonic()-began > 90:
                raise RuntimeError('bounded diagnostic timeout')
    except Exception as error:
        fault = type(error).__name__+': '+str(error)
    finally:
        state['started'] = False
        stop.set()
        until = time.monotonic()+.3
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        process.join(timeout=3)
        forced = process.is_alive()
        if forced:
            process.terminate()
            process.join(timeout=2)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        parent.close()

    for (seq, actual), (row, _) in zip(outputs, calls):
        if seq != row['seq'] or not equal_output(actual, row['data']['current_rebase']['output']):
            mismatches.append(seq)
    with (args.output/'received.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['seq', 'stamp_ns', 'received_ns'])
        writer.writerows(received)
    sent = list(csv.DictReader((args.output/'sent.csv').open()))
    tx, rx = {int(r['seq']) for r in sent}, {r[0] for r in received}
    source_gaps = [(b[1]-a[1])/1e6 for a, b in zip(received, received[1:])]
    tick_gaps = [(b-a)*1000 for a, b in zip(ticks, ticks[1:])]
    duration = float(sent[-1]['wall_s'])-float(sent[0]['wall_s']) if len(sent)>1 else None
    result = dict(mode=args.mode, calls=state['index'], sent=len(sent), received=len(received),
        missing=len(tx-rx), unexpected=len(rx-tx), duplicates=len(received)-len(rx),
        sent_hz=(len(sent)-1)/duration if duration else None,
        latency_ms=stats(latency), source_gap_ms=stats(source_gaps),
        source_gaps_over_50ms=sum(g>50 for g in source_gaps),
        parent_50hz_tick_interval_ms=stats(tick_gaps), compute_ms=stats(compute),
        mismatch_sequences=mismatches, source_unchanged=unchanged(),
        publisher_exitcode=process.exitcode, publisher_forced=forced, fault=fault,
        probe_sha256=sha(Path(__file__)), trace_sha256=sha(root/manifest['trace']),
        imu_history_queue_depth=64, scope='Single executor with64-sample IMU history queue; frozen current-rebase callbacks; no planner, guard, vision or actual sensors; no actuation')
    (args.output/'report.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
    (args.output/'durations.json').write_text(json.dumps(dict(compute_ms=compute, tick_intervals_ms=tick_gaps)), encoding='utf8')
    print(json.dumps(result))
    return 0 if fault is None and not forced and process.exitcode==0 and not mismatches and result['source_unchanged'] and state['index']==316 else 1


if __name__ == '__main__':
    raise SystemExit(main())

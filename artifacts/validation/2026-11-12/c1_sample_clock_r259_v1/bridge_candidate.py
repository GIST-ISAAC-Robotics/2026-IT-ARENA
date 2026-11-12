#!/usr/bin/env python3
"""격리 ROS domain에서 C1 원천 배열의 공통 decoder 수신까지 확인. 구동 출력 없음."""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import selectors
import signal
import subprocess
import time

from sample_clock import C1SampleClock as C1NativePreview

TOPIC = '/hardware_preview/c1/native'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        parser.error('seconds must be within 1..120')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '216':
        parser.error('preview requires ROS_LOCALHOST_ONLY=1 and isolated ROS_DOMAIN_ID=216')
    executable = args.executable.resolve(strict=True)
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from arena_autonomy.native_lidar_input import decode_measurements
    rclpy.init(args=[])
    node = Node('c1_native_preview')
    if node.get_parameter('use_sim_time').value:
        raise RuntimeError('hardware preview requires host ROS clock')
    converter = C1NativePreview(source_epoch=time.time_ns())
    publisher = node.create_publisher(String, TOPIC, qos_profile_sensor_data)
    counts = Counter()
    pending = Counter()
    intervals = []
    previous_receipt = None
    point_counts = []
    summaries = []

    def receive(message):
        decoded = decode_measurements(message.data, expected_time_basis='unverified_estimate')
        digest = hashlib.sha256(message.data.encode()).hexdigest()
        if not pending[digest]:
            raise RuntimeError('unexpected received payload')
        pending[digest] -= 1
        counts['received'] += 1
        counts['received_points'] += len(decoded.ranges_m)
        if decoded.time_basis != 'unverified_estimate':
            raise RuntimeError('time basis changed')

    subscription = node.create_subscription(String, TOPIC, receive, qos_profile_sensor_data)
    process = None
    error = None
    started = time.monotonic()
    stderr_path = args.output/'sdk.log'
    try:
        until = time.monotonic()+.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)
        with (args.output/'sdk_raw.jsonl').open('xb') as raw, (args.output/'converted.jsonl').open('x') as mapped, stderr_path.open('xb') as stderr:
            process = subprocess.Popen([str(executable), args.port, str(args.seconds)],
                stdout=subprocess.PIPE, stderr=stderr, bufsize=0)
            selector = selectors.DefaultSelector()
            selector.register(process.stdout, selectors.EVENT_READ)
            buffer = b''
            eof = False
            while not eof:
                if time.monotonic()-started > args.seconds+20:
                    raise TimeoutError('SDK preview exceeded duration')
                for key, _ in selector.select(.01):
                    block = os.read(key.fd, 65536)
                    if not block:
                        eof = True
                        break
                    raw.write(block)
                    buffer += block
                    if len(buffer) > 2_000_000:
                        raise ValueError('SDK line too large')
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        mono = time.monotonic_ns()
                        ros = node.get_clock().now().nanoseconds
                        batch = json.loads(line)
                        payload, status, due = converter.convert(batch,
                            receipt_monotonic_ns=mono, receipt_ros_ns=ros)
                        if payload is None:
                            counts['warmup_or_rejected'] += 1
                            mapped.write(json.dumps(dict(payload=None, diagnostic=status,
                                receipt_monotonic_ns=mono, receipt_ros_ns=ros), allow_nan=False)+'\n')
                            continue
                        # 추정 끝시각을 현재로 자르지 않는다. 최대5ms 후보 대기만 허용한다.
                        while time.monotonic_ns() < due:
                            rclpy.spin_once(node, timeout_sec=max(0., min(.001, (due-time.monotonic_ns())/1e9)))
                        publication_mono = time.monotonic_ns()
                        publication_ros = node.get_clock().now().nanoseconds
                        text = json.dumps(payload, separators=(',', ':'), allow_nan=False)
                        digest = hashlib.sha256(text.encode()).hexdigest()
                        pending[digest] += 1
                        message = String(); message.data = text
                        publisher.publish(message)
                        counts['published'] += 1
                        point_counts.append(len(payload['ranges_m']))
                        summaries.append(status)
                        if previous_receipt is not None:
                            intervals.append((mono-previous_receipt)/1e9)
                        previous_receipt = mono
                        mapped.write(json.dumps(dict(payload=payload, diagnostic=status,
                            receipt_monotonic_ns=mono, receipt_ros_ns=ros, publication_monotonic_ns=publication_mono, publication_ros_ns=publication_ros), allow_nan=False)+'\n')
                rclpy.spin_once(node, timeout_sec=0.)
            selector.close()
            if buffer:
                raise ValueError('truncated final SDK line')
            process.wait(timeout=5)
            if process.returncode:
                raise RuntimeError('SDK exit='+str(process.returncode))
            until = time.monotonic()+1.
            while time.monotonic() < until and any(pending.values()):
                rclpy.spin_once(node, timeout_sec=.01)
    except BaseException as exc:
        error = type(exc).__name__+': '+str(exc)
        raise
    finally:
        forced = False
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                forced = True; process.kill(); process.wait(timeout=3)
        import statistics
        log = stderr_path.read_text(errors='replace') if stderr_path.exists() else ''
        summary = dict(error=error, counts=dict(counts), clock=dict(converter.counts),
            pending=sum(pending.values()), sdk_returncode=process.returncode if process else None,
            forced_cleanup=forced, sensor_stop_ack='stop_result=0 motor_zero_result=0' in log,
            point_count_min=min(point_counts, default=0),
            point_count_median=statistics.median(point_counts) if point_counts else None,
            point_count_max=max(point_counts, default=0),
            observed_rate_hz=len(intervals)/sum(intervals) if intervals else None,
            maximum_receipt_interval_s=max(intervals, default=None),
            maximum_nominal_overlap_ns=max((s['nominal_overlap_ns'] for s in summaries), default=0),
            maximum_nominal_future_ns=max((s['nominal_end_after_receipt_ns'] for s in summaries), default=0),
            wall_elapsed_s=time.monotonic()-started, time_contract_verified=False,
            time_estimator=converter.estimator, topic=TOPIC, drive_output=False,
            mounting_calibrated=False, source_epoch=converter.epoch)
        (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps(summary))
    if not counts['received'] or sum(pending.values()) or not summary['sensor_stop_ack']:
        raise RuntimeError('preview delivery or sensor cleanup incomplete')


if __name__ == '__main__':
    main()

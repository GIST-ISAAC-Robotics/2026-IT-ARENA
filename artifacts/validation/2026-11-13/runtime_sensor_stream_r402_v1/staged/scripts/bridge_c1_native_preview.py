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

from arena_vehicle_interface.c1_native_preview import C1NativePreview
from arena_vehicle_interface.c1_sample_clock import C1SampleClock
from arena_vehicle_interface.host_clock_pair import sample_clocks
from sensor_stream_selection import add_stream_arguments, validate_stream

TOPIC = '/hardware_preview/c1/native'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scan-clock', choices=('sdk_nominal', 'tracking'), default='sdk_nominal',
                        help='tracking은 10 Hz Standard용 선택형 호스트 추정이며 취득 교정이 아닙니다')
    add_stream_arguments(parser, TOPIC)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        parser.error('seconds must be within 1..120')
    try: stream_selection = validate_stream(args.ros_domain, args.topic, os.environ)
    except ValueError as error: parser.error(str(error))
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
    tracking = args.scan_clock == 'tracking'
    converter = (C1SampleClock if tracking else C1NativePreview)(source_epoch=time.time_ns())
    publisher = node.create_publisher(String, args.topic, qos_profile_sensor_data)
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

    subscription = node.create_subscription(String, args.topic, receive, qos_profile_sensor_data)
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
                        clock_reads = None
                        if tracking:
                            mono, ros, clock_reads = sample_clocks(
                                time.monotonic_ns, lambda: node.get_clock().now().nanoseconds)
                        else:
                            # 기존 SDK nominal 기본의 수신 시각 의미는 유지한다.
                            mono = time.monotonic_ns()
                            ros = node.get_clock().now().nanoseconds
                        batch = json.loads(line)
                        result = converter.convert(batch, receipt_monotonic_ns=mono, receipt_ros_ns=ros)
                        if tracking:
                            payload, status, ready_ns = result
                        else:
                            payload, status = result
                            ready_ns = mono
                        publish_mono = publish_ros = None
                        counts['input_batches'] += 1
                        if payload is not None:
                            # 미래 끝 점을 현재로 잘라내지 않는다. 최대5ms만 기다리며 ROS도 처리한다.
                            while time.monotonic_ns() < ready_ns:
                                remaining = (ready_ns - time.monotonic_ns()) / 1e9
                                rclpy.spin_once(node, timeout_sec=max(0., min(.001, remaining)))
                            publish_mono = time.monotonic_ns()
                            publish_ros = node.get_clock().now().nanoseconds
                            if tracking and payload['acquired_at_s'][-1] > publish_ros / 1e9:
                                converter.reset('publication_ros_clock_step', status)
                                payload = None
                                counts['publication_clock_rejected'] += 1
                            else:
                                text = json.dumps(payload, separators=(',', ':'), allow_nan=False)
                                digest = hashlib.sha256(text.encode()).hexdigest()
                                pending[digest] += 1
                                message = String(); message.data = text
                                publisher.publish(message)
                                counts['published'] += 1
                                point_counts.append(len(payload['ranges_m']))
                        if payload is None:
                            counts['warmup_or_rejected'] += 1
                        summaries.append(status)
                        if previous_receipt is not None:
                            intervals.append((mono-previous_receipt)/1e9)
                        previous_receipt = mono
                        mapped.write(json.dumps(dict(payload=payload, diagnostic=status,
                            receipt_monotonic_ns=mono, receipt_ros_ns=ros, clock_reads=clock_reads,
                            ready_monotonic_ns=ready_ns, publication_monotonic_ns=publish_mono,
                            publication_ros_ns=publish_ros), allow_nan=False)+'\n')
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
            scan_clock=args.scan_clock,
            pending=sum(pending.values()), sdk_returncode=process.returncode if process else None,
            forced_cleanup=forced, sensor_stop_ack='stop_result=0 motor_zero_result=0' in log,
            point_count_min=min(point_counts, default=0),
            point_count_median=statistics.median(point_counts) if point_counts else None,
            point_count_max=max(point_counts, default=0),
            observed_rate_hz=len(intervals)/sum(intervals) if intervals else None,
            maximum_receipt_interval_s=max(intervals, default=None),
            maximum_nominal_overlap_ns=max((s['nominal_overlap_ns'] for s in summaries), default=0),
            maximum_nominal_future_ns=max((s['nominal_end_after_receipt_ns'] for s in summaries), default=0),
            maximum_requested_wait_ns=max((s.get('publication_wait_ns', 0) for s in summaries), default=0),
            wall_elapsed_s=time.monotonic()-started, time_contract_verified=False,
            time_estimator=converter.estimator, topic=args.topic, stream_selection=stream_selection, drive_output=False,
            mounting_calibrated=False, source_epoch=converter.epoch)
        if tracking:
            summary['nominal_clock'] = dict(converter.raw.counts)
        (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps(summary))
    if not counts['received'] or sum(pending.values()) or not summary['sensor_stop_ack']:
        raise RuntimeError('preview delivery or sensor cleanup incomplete')


if __name__ == '__main__':
    main()

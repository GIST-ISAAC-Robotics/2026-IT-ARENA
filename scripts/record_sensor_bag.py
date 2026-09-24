#!/usr/bin/env python3
"""명시적 허용 목록의 원시 CDR·수신 시각·실행 매개변수를 보존한다. 제어 출력 없음."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import platform
import queue
import shutil
import signal
import sys
import threading
import time

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src/arena_vehicle_interface'))
from arena_vehicle_interface.bag_contract import TOPICS, REQUIRED, sha256_file, update_digest, dds_metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-wall-seconds', type=float, default=1200.)
    parser.add_argument('--max-raw-gib', type=float, default=6.)
    args = parser.parse_args()
    if not 1 <= args.max_wall_seconds <= 7200 or not .01 <= args.max_raw_gib <= 20:
        raise ValueError('recording limits out of range')
    output = args.output.resolve()
    if not output.is_relative_to(REPO / 'artifacts'):
        raise ValueError('output must be under artifacts')
    if shutil.disk_usage(REPO).free < 10 * 1024**3:
        raise ValueError('at least 10 GiB free space required')
    output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from rclpy.signals import SignalHandlerOptions
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('sensor_bag_recorder', parameter_overrides=[Parameter('use_sim_time', value=True)])
    types = {topic: get_message(type_name) for topic, type_name in TOPICS.items()}
    items = queue.Queue(maxsize=5000)
    dropped, stats, digests = Counter(), {}, {}
    pending_bytes = total_bytes = 0
    byte_lock = threading.Lock()
    finished, ready, requested = threading.Event(), threading.Event(), threading.Event()
    errors = []
    regressions = preclock = 0
    last_ros = None
    began = time.monotonic()
    parameters = {}
    def stop(*_):
        requested.set()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def writer_worker():
        nonlocal pending_bytes
        writer = None
        try:
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(output/'bag'), storage_id='mcap',
                                                storage_preset_profile='zstd_fast'),
                        rosbag2_py.ConverterOptions('', ''))
            for index, (topic, type_name) in enumerate(TOPICS.items()):
                writer.create_topic(rosbag2_py.TopicMetadata(id=index, name=topic, type=type_name, serialization_format='cdr'))
                digests[topic] = hashlib.sha256()
                stats[topic] = dict(count=0, bytes=0, first_receive_ns=None, last_receive_ns=None,
                                    first_source_ns=None, last_source_ns=None, source_regressions=0)
            with (output/'receipt.jsonl').open('w', encoding='utf-8') as audit:
                ready.set()
                while not finished.is_set() or not items.empty():
                    try:
                        topic, payload, ros_ns, wall_ns, info = items.get(timeout=.1)
                    except queue.Empty:
                        continue
                    try:
                        message = deserialize_message(payload, types[topic])
                        stamp = message.header.stamp if hasattr(message, 'header') else None
                        source_ns = None if stamp is None else stamp.sec*1_000_000_000+stamp.nanosec
                        writer.write(topic, payload, ros_ns)
                        update_digest(digests[topic], payload, ros_ns)
                        record = stats[topic]
                        record['count'] += 1
                        record['bytes'] += len(payload)
                        if record['first_receive_ns'] is None:
                            record['first_receive_ns'], record['first_source_ns'] = ros_ns, source_ns
                        if source_ns is not None and record['last_source_ns'] is not None and source_ns < record['last_source_ns']:
                            record['source_regressions'] += 1
                        record['last_receive_ns'], record['last_source_ns'] = ros_ns, source_ns
                        audit.write(json.dumps(dict(topic=topic, index=record['count'], source_ns=source_ns,
                            receive_ros_ns=ros_ns, receive_monotonic_ns=wall_ns,
                            writer_monotonic_ns=time.monotonic_ns(), serialized_bytes=len(payload),
                            **info), separators=(',', ':'))+'\n')
                    finally:
                        with byte_lock:
                            pending_bytes -= len(payload)
                        items.task_done()
            writer.close()
            writer = None
        except Exception as error:
            errors.append(repr(error))
            requested.set()
            ready.set()
        finally:
            if writer is not None:
                writer.close()

    worker = threading.Thread(target=writer_worker, name='bag_writer')
    worker.start()
    if not ready.wait(15) or errors:
        finished.set()
        worker.join(timeout=15)
        node.destroy_node()
        rclpy.shutdown()
        (output/'manifest.json').write_text(json.dumps(dict(schema=1, complete=False,
            error=errors or ['writer startup timeout'])), encoding='utf-8')
        raise RuntimeError('bag writer failed to start: ' + str(errors))

    def receive(topic):
        def callback(payload, message_info):
            nonlocal pending_bytes, total_bytes, last_ros, regressions, preclock
            wall_ns = time.monotonic_ns()
            ros_ns = node.get_clock().now().nanoseconds
            if ros_ns <= 0:
                preclock += 1
                return
            if last_ros is not None and ros_ns < last_ros:
                regressions += 1
                requested.set()
                return
            last_ros = ros_ns
            size = len(payload)
            with byte_lock:
                if pending_bytes + size > 64 * 1024**2 or items.full():
                    dropped[topic] += 1
                    return
                pending_bytes += size
                total_bytes += size
            info = dds_metadata(message_info)
            items.put_nowait((topic, bytes(payload), ros_ns, wall_ns, info))
            if total_bytes > args.max_raw_gib*1024**3:
                errors.append('raw byte limit reached; partial recording retained')
                requested.set()
        return callback

    for topic in TOPICS:
        # BEST_EFFORT는 원래 센서 송신 QoS와 호환되며 재전송으로 제어를 막지 않는다.
        node.create_subscription(types[topic], topic, receive(topic),
                                 QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True)
    clients = {name: AsyncParameterClient(node, name) for name in ('local_pursuit', 'lidar_safety')}
    requests = {}
    last_disk_check = 0.
    (output/'ready.json').write_text(json.dumps({'ready': True, 'pid': os.getpid()}))
    try:
        while rclpy.ok() and not requested.is_set():
            rclpy.spin_once(node, timeout_sec=.01)
            for name, client in clients.items():
                if name in parameters:
                    continue
                if name not in requests and client.services_are_ready():
                    requests[name] = ('list', client.list_parameters(), None)
                if name in requests:
                    phase, future, names = requests[name]
                    if future.done():
                        if phase == 'list':
                            names = future.result().result.names
                            requests[name] = ('get', client.get_parameters(names), names)
                        else:
                            parameters[name] = {n: parameter_value_to_python(p) for n, p in zip(names, future.result().values)}
            if time.monotonic() - began > args.max_wall_seconds:
                errors.append('wall duration limit reached; partial recording retained')
                break
            if time.monotonic() - last_disk_check > 1.:
                last_disk_check = time.monotonic()
                if shutil.disk_usage(output).free < 2 * 1024**3:
                    errors.append('disk reserve reached; partial recording retained')
                    break
    except Exception as error:
        errors.append(repr(error))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        finished.set()
        worker.join(timeout=60)
        if worker.is_alive():
            errors.append('writer did not finish within 60 seconds')
        (output/'parameters.json').write_text(json.dumps(parameters, indent=2), encoding='utf-8')
        for topic, stat in stats.items():
            stat['cdr_sha256'] = digests[topic].hexdigest()
        files = {str(p.relative_to(output)): sha256_file(p) for p in sorted(output.rglob('*'))
                 if p.is_file() and p.name != 'manifest.json'}
        complete = (not errors and not dropped and not regressions and len(parameters) == 2 and
                    all(stats.get(t, {}).get('count', 0) > 0 for t in REQUIRED))
        manifest = dict(schema=1, complete=complete, topics=TOPICS, stats=stats, queue_drops=dict(dropped),
                        clock_regressions=regressions, preclock_skipped=preclock, error=errors,
                        wall_duration_s=time.monotonic()-began, raw_bytes=total_bytes, files_sha256=files,
                        storage='mcap/zstd_fast', time_basis='bag timestamp = recorder ROS receipt, headers unchanged',
                        runtime={'platform': platform.platform(), 'machine': platform.machine(),
                                 'python': platform.python_version(), 'ros_distro': os.environ.get('ROS_DISTRO')},
                        loss_scope='queue losses counted; DDS source counters retained, total upstream losses not certified',
                        source_sha256={str(p.relative_to(REPO)): sha256_file(p) for p in [Path(__file__).resolve(),
                            REPO/'src/arena_vehicle_interface/arena_vehicle_interface/bag_contract.py']})
        (output/'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
        print(json.dumps({'complete': complete, 'error': errors, 'raw_bytes': total_bytes}), flush=True)
    return 0 if complete else 1


if __name__ == '__main__':
    raise SystemExit(main())

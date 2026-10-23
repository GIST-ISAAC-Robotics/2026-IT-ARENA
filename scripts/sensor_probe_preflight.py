#!/usr/bin/env python3
"""센서 기록 전 설치 API/경로의 읽기 전용 점검. ROS 노드·장치·writer를 열지 않는다."""
import argparse
import importlib
import json
import os
from pathlib import Path
import platform
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
from sensor_recording_profile import select_profile
from rosbag_writer_compat import check_writer_api, topic_metadata


def inspect_environment(capture, clock_mode, topics, *, root=ROOT, loader=importlib.import_module,
                        disk_usage=shutil.disk_usage, version=None):
    profile = select_profile('sensor_probe', clock_mode, topics, 'drive_motor_shaft')
    root = Path(root).resolve()
    capture = Path(capture).resolve()
    checks = {}
    version = sys.version_info[:3] if version is None else version
    checks['python_version'] = dict(ok=version >= (3, 10), observed=list(version), minimum='3.10')
    within = capture.is_relative_to(root/'artifacts') and capture != root/'artifacts'
    fresh = not capture.exists()
    checks['new_artifacts_directory'] = dict(ok=within and fresh, inside_artifacts=within, absent=fresh)
    ancestor = capture.parent
    while not ancestor.exists() and ancestor != ancestor.parent:
        ancestor = ancestor.parent
    try:
        free = disk_usage(ancestor).free
        checks['free_space'] = dict(ok=free >= 10*1024**3, free_bytes=free, required_bytes=10*1024**3)
        checks['parent_writable_hint'] = dict(ok=ancestor.is_dir() and os.access(ancestor, os.W_OK),
                                               advisory_only=True, write_attempted=False)
    except OSError as error:
        checks['free_space'] = dict(ok=False, error_type=type(error).__name__)
    modules = {}
    for name in ('rclpy', 'rclpy.serialization', 'rosbag2_py', 'rosidl_runtime_py.utilities', 'yaml'):
        try:
            modules[name] = loader(name)
            checks['import:'+name] = dict(ok=True)
        except (ImportError, OSError, RuntimeError) as error:
            checks['import:'+name] = dict(ok=False, error_type=type(error).__name__)
    bag = modules.get('rosbag2_py')
    if bag is not None:
        try:
            check_writer_api(bag)
            checks['writer_api'] = dict(ok=True, writer_instantiated=False)
        except (AttributeError, TypeError, RuntimeError) as error:
            checks['writer_api'] = dict(ok=False, error_type=type(error).__name__)
        for name in ('writers', 'readers'):
            query = getattr(bag, 'get_registered_'+name, None)
            try:
                supported = sorted(query()) if callable(query) else None
                checks['mcap_'+name] = dict(ok=supported is not None and 'mcap' in supported,
                                            observed=supported, queried=callable(query))
            except (RuntimeError, TypeError, OSError) as error:
                checks['mcap_'+name] = dict(ok=False, error_type=type(error).__name__)
        try:
            for index, (topic, type_name) in enumerate(profile['topics'].items()):
                metadata = topic_metadata(bag, index, topic, type_name)
                if (metadata.name, metadata.type, metadata.serialization_format) != (topic, type_name, 'cdr'):
                    raise ValueError('metadata fields differ')
            checks['topic_metadata'] = dict(ok=True, uses_id=hasattr(bag.TopicMetadata, 'id'))
        except (AttributeError, TypeError, RuntimeError, ValueError) as error:
            checks['topic_metadata'] = dict(ok=False, error_type=type(error).__name__)
    utilities = modules.get('rosidl_runtime_py.utilities')
    serialization = modules.get('rclpy.serialization')
    if utilities is not None and serialization is not None:
        for topic, type_name in profile['topics'].items():
            try:
                message_type = utilities.get_message(type_name)
                original = message_type()
                payload = serialization.serialize_message(original)
                restored = serialization.deserialize_message(payload, message_type)
                if restored != original:
                    raise ValueError('empty-message field roundtrip differs')
                checks['message:'+topic] = dict(ok=True, type=type_name, empty_cdr_bytes=len(payload))
            except (ImportError, AttributeError, TypeError, ValueError, RuntimeError) as error:
                checks['message:'+topic] = dict(ok=False, type=type_name, error_type=type(error).__name__)
    return dict(schema='arena_sensor_probe_preflight_v1', checks_passed=all(c['ok'] for c in checks.values()),
                checks=checks, profile=profile['name'], recorder_clock_mode=clock_mode,
                topics=profile['topics'], capture_path=str(capture),
                runtime=dict(python=platform.python_version(), machine=platform.machine(),
                             ros_distro=os.environ.get('ROS_DISTRO'),
                             rmw_requested=os.environ.get('RMW_IMPLEMENTATION')),
                ros_node_started=False, device_opened=False, writer_opened=False,
                directory_created=False, packages_installed=False, transport_config_changed=False,
                mcap_write_verified=False, dds_delivery_verified=False,
                sensor_time_verified=False, hardware_verified=False, ready_to_drive=False,
                limitations=[
                    'Plugin registration is not an actual MCAP open/write or compression check.',
                    'Requested RMW/environment are not an inspection of an active participant.',
                    'Free space and access hint can change; the recorder repeats its own checks.',
                    'No producer, QoS matching, clock source, sensor calibration or high-rate delivery is tested.',
                ])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--clock-mode', choices=('system', 'simulation'), required=True)
    parser.add_argument('--sensor-topic', action='append', required=True)
    parser.add_argument('--output', type=Path, help='선택 보고서만 새 파일로 저장; 없으면 stdout만 사용')
    args = parser.parse_args(argv)
    if args.output is not None and args.output.exists():
        raise FileExistsError(args.output)
    report = inspect_environment(args.capture, args.clock_mode, args.sensor_topic)
    if args.output is not None:
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write('\n')
    print(json.dumps(report, allow_nan=False))
    return 0 if report['checks_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

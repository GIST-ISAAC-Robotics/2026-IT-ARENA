#!/usr/bin/env python3
"""A3용 명시적 소스/원시 기록 묶음. 결과 archive는 Git 제외 build에만 저장한다."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ('arena_autonomy', 'arena_vehicle_interface')
SCRIPTS = ('replay_sensor_bag.py', 'replay_observer.py', 'audit_sensor_bag.py', 'audit_replay_delivery.py',
           'summarize_controller_performance.py', 'run_jetson_replay_trial.py',
           'check_vision_worker_equivalence.py', 'jetson_run_vision_process.sh',
           'analyze_wheel_delivery.py', 'jetson_run_wheel_dispatch.sh',
           'replay_transport.py', 'dds_transport_probe.cpp', 'replay_imu_probe.py',
           'replay_timing.py', 'analyze_imu_delivery.py', 'jetson_run_timing_cost.sh')
TESTS = ('test_replay_imu_probe.py', 'test_replay_transport.py', 'test_node_lifecycle_modes.py', 'test_wheel_delivery_analysis.py', 'test_replay_continuity.py', 'test_vision_process.py', 'test_signal_mask.py', 'test_replay_observer.py', 'test_bag_contract.py', 'test_bag_reader_compat.py', 'test_timing_probe.py', 'test_queued_log.py',
         'test_lidar_motion.py', 'test_lidar_motion_ros.py', 'test_lidar_safety.py',
         'test_local_path_range_noise.py', 'test_replay_timing.py', 'test_imu_delivery_analysis.py')
DDS_TOOLS = ('scripts/replay_sensor_bag.py', 'scripts/replay_observer.py',
             'scripts/run_jetson_replay_trial.py', 'scripts/replay_transport.py',
             'scripts/dds_transport_probe.cpp', 'scripts/replay_imu_probe.py', 'tests/test_replay_transport.py')
IMU_PROBE_TOOLS = ('scripts/replay_sensor_bag.py', 'scripts/run_jetson_replay_trial.py',
                   'scripts/replay_imu_probe.py', 'tests/test_replay_imu_probe.py')


def with_replay_dependencies(files):
    """작은 재생기 갱신 묶음에서도 새 import 모듈을 빠뜨리지 않는다."""
    result = list(files)
    dependency = ROOT/'scripts/replay_timing.py'
    if ROOT/'scripts/replay_sensor_bag.py' in result and dependency not in result:
        result.append(dependency)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, default=ROOT/'build/jetson_a3_20260928.tar.gz')
    parser.add_argument('--manifest', type=Path, default=ROOT/'artifacts/validation/2026-09-28/jetson_a3/bundle_manifest.json')
    parser.add_argument('--without-recording', action='store_true', help='기존 원격 기록을 유지하는 소스 갱신 묶음')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--dds-tools-only', action='store_true',
                        help='제어기/기록을 제외한 DDS 재생·검사 도구와 수신자 의존 파일만 묶는다')
    group.add_argument('--imu-trace-tools-only', action='store_true',
                       help='IMU 발행 계측 재생기/실행기와 수신자 의존 파일만 묶는다')
    group.add_argument('--imu-probe-tools-only', action='store_true',
                       help='최소 IMU 수신자·재생 연결·시험과 필수 import 의존 파일을 묶는다')
    args = parser.parse_args()
    archive, manifest = args.archive.resolve(), args.manifest.resolve()
    if not archive.is_relative_to(ROOT/'build') or not manifest.is_relative_to(ROOT/'artifacts'):
        raise ValueError('Archive must be under build and manifest under artifacts')
    if archive.exists() or manifest.exists():
        raise FileExistsError('Existing bundle is not overwritten')
    subset = (IMU_PROBE_TOOLS if args.imu_probe_tools_only else ('scripts/replay_sensor_bag.py', 'scripts/run_jetson_replay_trial.py', 'scripts/replay_imu_probe.py')
              if args.imu_trace_tools_only else DDS_TOOLS if args.dds_tools_only else ())
    files = [ROOT/name for name in subset]
    if not subset:
        for package in PACKAGES:
            files.extend(p for p in (ROOT/'src'/package).rglob('*') if p.is_file()
                         and '__pycache__' not in p.parts and p.suffix != '.pyc')
        files += [ROOT/'scripts'/name for name in SCRIPTS]
        files += [ROOT/'tests'/name for name in TESTS]
    files = with_replay_dependencies(files)
    recording = ROOT/'artifacts/validation/2026-09-24/record_replay/capture_v2/sensor_recording'
    includes_recording = not (args.without_recording or subset)
    if includes_recording:
        files += [p for p in recording.rglob('*') if p.is_file()]
    inventory = {str(p.relative_to(ROOT)).replace('\\', '/'): dict(size=p.stat().st_size,
                 sha256=hashlib.file_digest(p.open('rb'), 'sha256').hexdigest()) for p in sorted(files)}
    archive.parent.mkdir(exist_ok=True)
    with tarfile.open(archive, 'x:gz', compresslevel=1) as tar:
        for p in files:
            tar.add(p, arcname=p.relative_to(ROOT).as_posix(), recursive=False)
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps(dict(files=inventory, archive_size=archive.stat().st_size,
        includes_recording=includes_recording,
        bundle_mode='imu-probe-tools-only' if args.imu_probe_tools_only else 'imu-trace-tools-only' if args.imu_trace_tools_only else 'dds-tools-only' if args.dds_tools_only else 'source',
        archive_sha256=hashlib.file_digest(archive.open('rb'), 'sha256').hexdigest(),
        scope='Private PC-to-user-Jetson transfer; archive/raw sensor data excluded from Git'), indent=2))
    print(json.dumps({'files': len(files), 'archive_bytes': archive.stat().st_size,
                      'manifest': str(manifest)}))


if __name__ == '__main__':
    main()

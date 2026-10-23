#!/usr/bin/env python3
"""센서 점검 전용 묶음의 해시/내용 검사. 추출·전송·명령/ROS 실행 없음."""
import argparse
import ast
import hashlib
import json
from pathlib import Path, PurePosixPath
import tarfile

LEGACY_EXPECTED = {
    'scripts/record_sensor_bag.py', 'scripts/recording_guard.py', 'scripts/sensor_recording_profile.py',
    'scripts/rosbag_writer_compat.py', 'scripts/audit_sensor_probe.py', 'scripts/audit_sensor_bag.py',
    'scripts/verify_sensor_probe_bundle.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/__init__.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/bag_contract.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py',
    'tests/test_sensor_recording_profile.py', 'tests/test_sensor_probe_audit.py',
    'tests/test_rosbag_writer_compat.py', 'tests/test_recording_guard.py',
    'docs/hardware/SENSOR_PROBE_RUNBOOK.md',
}
V2_EXPECTED = LEGACY_EXPECTED | {
    'scripts/sensor_probe_preflight.py', 'scripts/analyze_sensor_probe_timing.py',
    'tests/test_sensor_probe_preflight.py', 'tests/test_sensor_probe_timing.py',
}
V3_EXPECTED = V2_EXPECTED | {'scripts/inspect_sensor_probe_inputs.py', 'tests/test_sensor_probe_inputs.py'}
EXPECTED = V3_EXPECTED | {'scripts/sensor_probe_session.py', 'tests/test_sensor_probe_session.py'}
INVENTORIES = {1: LEGACY_EXPECTED, 2: V2_EXPECTED, 3: V3_EXPECTED, 4: EXPECTED}


def digest(stream):
    h = hashlib.sha256()
    for data in iter(lambda: stream.read(1024*1024), b''):
        h.update(data)
    return h.hexdigest()


def verify(archive, manifest):
    if (manifest.get('bundle_mode') != 'sensor-probe-tools-only'
            or manifest.get('includes_recording') is not False
            or manifest.get('includes_follow_traces') is not False):
        raise ValueError('explicit sensor-only bundle without records/traces required')
    version = manifest.get('sensor_probe_tools_version', 1)
    if type(version) is not int or version not in INVENTORIES:
        raise ValueError('unsupported sensor probe tools version')
    inventory = INVENTORIES[version]
    expected = manifest.get('files')
    if type(expected) is not dict or set(expected) != inventory:
        raise ValueError('exact minimal file inventory required')
    archive = Path(archive)
    if not 0 < archive.stat().st_size <= 8*1024*1024:
        raise ValueError('archive exceeds bounded tool-only size')
    with archive.open('rb') as stream:
        matching = digest(stream) == manifest['archive_sha256']
    matching = matching and archive.stat().st_size == manifest['archive_size']
    seen, problems, parsed = set(), [], []
    total = 0
    with tarfile.open(archive, 'r:gz') as bundle:
        for member in bundle:
            path = PurePosixPath(member.name)
            total += member.size
            if (not member.isfile() or path.is_absolute() or '..' in path.parts
                    or '\\' in member.name or ':' in member.name or member.name in seen
                    or member.name not in inventory or not 0 <= member.size <= 2*1024*1024
                    or total > 8*1024*1024):
                raise ValueError('unsafe, duplicate, unexpected or oversized archive member')
            seen.add(member.name)
            with bundle.extractfile(member) as stream:
                data = stream.read(member.size+1)
            spec = expected[member.name]
            if (type(spec) is not dict or type(spec.get('size')) is not int
                    or spec['size'] != len(data) or hashlib.sha256(data).hexdigest() != spec.get('sha256')):
                problems.append(dict(file=member.name, reason='hash_or_size_differs'))
            if path.suffix == '.py':
                try:
                    ast.parse(data, filename=member.name, feature_version=(3, 10))
                    parsed.append(member.name)
                except (SyntaxError, ValueError):
                    problems.append(dict(file=member.name, reason='python310_syntax'))
    missing = sorted(inventory-seen)
    return dict(passed=matching and not missing and not problems, members=len(seen),
                archive_hash_matches=matching, missing=missing, problems=problems,
                sensor_probe_tools_version=version,
                python310_parsed=len(parsed), extracted=False, transferred=False,
                python310_runtime_verified=False, jetson_verified=False, hardware_verified=False,
                ready_to_drive=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = verify(args.archive, json.loads(args.manifest.read_text()))
    with args.output.open('x', encoding='utf-8') as out:
        json.dump(result, out, indent=2)
        out.write('\n')
    print(json.dumps(result))
    raise SystemExit(0 if result['passed'] else 1)

#!/usr/bin/env python3
"""r404 방식의 최신 운영 소스 목록/후보 ZIP. 장치 접근·실행·설치 없음."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

PREFIX = 'src/arena_autonomy/arena_autonomy/'
REQUIRED = tuple(PREFIX + name for name in (
    'race_runtime_policy/current_route_priority.py', 'local_follow.py',
    'known_structure.py', 'moving_pass.py', 'bypass_entry_trial.py',
    'independent_guard_route.py', 'g1_g2_node.py', 'lidar_safety.py',
    'race_ops_node.py', 'race_ops_manual.py', 'development_jog.py',
)) + tuple('scripts/' + name for name in (
    'run_runtime_actuation_host.py', 'runtime_actuation_link.py',
    'runtime_installed_profile.py', 'runtime_installed_limits.py',
    'steering_zero_profile.py', 'race_operator.py', 'development_jog.py',
    'run_hardware_sensor_preview.py', 'bridge_c1_native_preview.py',
    'bridge_esp32_imu_preview.py', 'bridge_sensor_link_preview.py',
    'bridge_b0390_preview.py', 'stream_c1_native_preview.cpp',
    'probe_g1_hardware_shadow.py', 'build_runtime_drive.sh', 'build_runtime_preview.sh',
)) + ('src/arena_autonomy/setup.py', 'src/arena_vehicle_interface/setup.py',
      'src/arena_vehicle_interface/arena_vehicle_interface/steering_cad_table.py')
TEMPLATE = 'artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml'
# 운영 경로에서 읽는 공개 리소스만 명시한다. config/ 전체나 장착 JSON을 수집하지 않는다.
PUBLIC_RESOURCES = {
    'config/hardware/g1_input_shadow_v1.yaml': 'preview default; observation-only, not installed geometry',
    'config/perception/vehicle_marker_approx_b0390.json': 'optional nominal/CAD profile; accuracy unverified, default OFF',
    'config/perception/vehicle_marker_gist49_calibration_only.json': 'optional calibration-only template; not runtime occupancy',
    'config/perception/vehicle_marker_samples_template.json': 'empty measurement template; nulls remain null',
    'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml': 'explicit preview geometry input; CAD, not measured mount',
    'src/arena_description/config/b0390_c1_cad_20261113_final_nowire.yaml': 'nominal sensor specification/reference; not automatically selected',
}
REQUIRED += tuple(PUBLIC_RESOURCES) + ('handover/runtime_profile.template.yaml',)
EXTENSIONS = {'.py', '.cpp', '.hpp', '.h', '.xml', '.cfg', '.sh'}
SKIP = {'build', 'install', 'log', '__pycache__', '.git', '.venv', 'private', '.private'}


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def collect(repo):
    """정적 목록이 아니라 패키지 전체를 수집해 새 하위 모듈도 자동 포함한다."""
    files = {}
    for folder in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts'):
        for path in (repo / folder).rglob('*'):
            relative = path.relative_to(repo)
            if (path.is_file() and not path.is_symlink() and not SKIP.intersection(relative.parts)
                    and (path.suffix in EXTENSIONS or path.name in ('CMakeLists.txt', 'sdkconfig.defaults')
                         or path.parent.name == 'resource')):
                files[relative.as_posix()] = path.read_bytes()
    for name in PUBLIC_RESOURCES:
        path = repo / name
        if path.is_file() and not path.is_symlink():
            files[name] = path.read_bytes()
    # 원 r401은 실측/최신 선택이 아닌 참고 템플릿으로 명확하게 이름을 바꾼다.
    if (repo / TEMPLATE).is_file():
        files['handover/runtime_profile.template.yaml'] = (repo / TEMPLATE).read_bytes()
    return files


def inventory(files):
    missing = sorted(set(REQUIRED) - files.keys())
    return dict(schema='runtime_handover_source_candidate_r491_v1',
                files={name: sha(raw) for name, raw in sorted(files.items())},
                required_missing=missing, source_complete=not missing,
                public_resources={name: dict(purpose=purpose, automatically_selected=False)
                                  for name, purpose in PUBLIC_RESOURCES.items()},
                installed_values=dict(assembly_id=None, servo_zero_count=None,
                                      board_profile=None, installed_profile=None,
                                      selected_runtime_params=None, clock_policy=None,
                                      motor_feedback_config=None, runtime_port=None),
                source_only=True, deployment_complete=False, firmware_written=False,
                device_io=False, arm_or_drive=False,
                ground_instruction=dict(initial_target_mps=.05, command_ceiling_mps=.1,
                                        measured_overspeed_cutoff_verified=False),
                air_instruction=dict(absolute_ceiling_kmh=13., enabled=False,
                                     measured_overspeed_cutoff_verified=False),
                profile_template_source=TEMPLATE)


def write_bundle(files, destination):
    """별도 이름의 새 ZIP만 생성한다. 바이너리/장착값/인증정보는 넣지 않는다."""
    with zipfile.ZipFile(destination, 'x', zipfile.ZIP_DEFLATED) as archive:
        for name, raw in sorted(files.items()):
            archive.writestr(name, raw)
        archive.writestr('source_manifest.json', json.dumps(inventory(files), indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--archive', action='store_true', help='r489 공통 이관/주 담당 동결 뒤에만 명시')
    args = parser.parse_args()
    artifact = Path(__file__).resolve().parents[2]
    output = args.output.resolve()
    if not output.is_relative_to(artifact) or output == artifact or output.exists():
        parser.error('fresh output directory under r491 artifact required')
    files = collect(args.repo.resolve())
    report = inventory(files)
    output.mkdir(parents=True)
    if args.archive and report['required_missing']:
        report['archive_error'] = 'required source not integrated; no ZIP generated'
    elif args.archive:
        target = output / 'runtime_handover_sources.zip'
        write_bundle(files, target)
        report['archive_sha256'] = sha(target.read_bytes())
    (output / 'manifest.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(files=len(files), missing=report['required_missing'],
                          archive_created='archive_sha256' in report,
                          physical_values_set=False)))
    return 2 if 'archive_error' in report else 0


if __name__ == '__main__':
    raise SystemExit(main())

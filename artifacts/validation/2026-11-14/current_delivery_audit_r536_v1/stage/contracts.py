"""단일 재생의 로컬 경로·명시 profile 계약. 네트워크/ROS import 없음."""
import copy
import hashlib
import json
from pathlib import Path

INPUT_SHA = 'c2df9fe72436204d933eb0c078fde972053f8086521ce0701cfce93f6e332ad1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_snapshot(root, expected_manifest):
    root = Path(root).resolve(strict=True)
    manifest_path = root/'source_manifest.json'
    if sha(manifest_path) != expected_manifest:
        raise ValueError('explicit source manifest hash mismatch')
    manifest = json.loads(manifest_path.read_bytes())
    for name, expected in manifest['files'].items():
        path = (root/name).resolve(strict=True)
        if not path.is_relative_to(root) or sha(path) != expected:
            raise ValueError('source changed or outside assigned root: '+name)
    installed = {str(p.relative_to(root)):sha(p) for p in (root/'install').rglob('*')
                 if p.is_file() and p.suffix in ('.py','.so')}
    if not installed:
        raise ValueError('no installed Python/native files')
    return dict(source_manifest_sha256=expected_manifest,source_files=len(manifest['files']),installed=installed)


def prepared_profile(profile, output):
    result = copy.deepcopy(profile)
    controller = result['local_pursuit']['ros__parameters']
    if controller.get('g1_trace_detail') != 'compact_v1':
        raise ValueError('explicit compact_v1 profile required; no silent override')
    if controller.get('g1_complete_plan_trace') is not False:
        raise ValueError('complete plan trace must be explicitly false')
    if controller.get('race_control_mode') != 'legacy':
        raise ValueError('frozen replay uses legacy start; no B3 substitution')
    for name in ('local_pursuit','lidar_safety'):
        params = result[name]['ros__parameters']
        if (params.get('use_sim_time') is not True or params.get('execution_environment') != 'default'
                or params.get('lidar_input_representation') != 'native_simulation_v1'):
            raise ValueError('simulation-only profile required: '+name)
        if params.get('motion_imu_frame_id') != 'r506_reconstructed_sim_imu':
            raise ValueError('original producer frame required')
    controller['g1_trace_path'] = str(Path(output).resolve()/'controller_trace.jsonl')
    return result


def assert_installed_package(package_file, root):
    root, path = Path(root).resolve(strict=True), Path(package_file).resolve(strict=True)
    if not path.is_relative_to(root/'install'):
        raise ValueError('import is outside the selected installed root: '+str(path))


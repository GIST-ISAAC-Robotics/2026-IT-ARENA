"""PC에서 profile 선택본만 준비한다. 프로세스/ROS/장치 실행은 하지 않는다."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path, PurePosixPath
import uuid

SIDECAR = 'guard_status_contexts.jsonl'
MODES = ('full', 'compact_v1')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def target_path(value):
    path = PurePosixPath(value)
    if not path.is_absolute() or '..' in path.parts or path == PurePosixPath('/'):
        raise ValueError('a non-root absolute target Linux path is required')
    return path


def overlay(profile, mode, result_path):
    if mode not in MODES:
        raise ValueError('unknown guard detail mode')
    result = copy.deepcopy(profile)
    if result['local_pursuit']['ros__parameters'].get('g1_trace_detail') != 'compact_v1':
        raise ValueError('select an explicit planner compact profile; no implicit override')
    guard = result['lidar_safety']['ros__parameters']
    guard['safety_status_detail'] = mode
    guard['safety_status_evidence_path'] = str(target_path(result_path)/SIDECAR) if mode == 'compact_v1' else ''
    return result


def prepare(base_profile, package_parent, target_results_root, mode='full'):
    target_root = target_path(target_results_root)
    profile = json.loads(Path(base_profile).read_bytes())
    run_id = 'r547_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + uuid.uuid4().hex
    target_result = target_root/run_id
    selected = overlay(profile, mode, target_result)
    package = Path(package_parent)/run_id
    package.mkdir(parents=True, exist_ok=False)
    profile_path = package/'profile.json'
    profile_path.write_text(json.dumps(selected, indent=2)+'\n', encoding='utf-8')
    selection = dict(schema='r547_guard_status_recipe_v1', run_id=run_id,
        requested_guard_detail=mode, target_result=str(target_result),
        target_sidecar=str(target_result/SIDECAR) if mode == 'compact_v1' else None,
        base_profile_sha256=sha(base_profile), selected_profile_sha256=sha(profile_path),
        allowed_profile_changes=['lidar_safety.ros__parameters.safety_status_detail',
                                 'lidar_safety.ros__parameters.safety_status_evidence_path'],
        compact_requires_adopted_installed_r543=mode == 'compact_v1',
        launched=False, target_path_checked_remotely=False)
    (package/'selection.json').write_text(json.dumps(selection, indent=2)+'\n', encoding='utf-8')
    return package


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True, type=Path)
    parser.add_argument('--package-parent', required=True, type=Path)
    parser.add_argument('--target-results-root', required=True)
    parser.add_argument('--guard-detail', choices=MODES, default='full')
    args = parser.parse_args()
    print(prepare(args.profile, args.package_parent, args.target_results_root, args.guard_detail))


if __name__ == '__main__':
    main()

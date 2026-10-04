"""보존한 8사례에 새 관측 판정만 재적용. ROS/Gazebo 실행이나 원본 갱신 없음."""
import ast
import hashlib
import json
import math
from pathlib import Path
import zipfile

REPO = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
COLLECTION = REPO/'artifacts/validation/2026-10-05/g0_parent_finish_v1/collection_report.json'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def main():
    report_path, snapshot_path = OUT/'reassessment.json', OUT/'source_snapshot.zip'
    if report_path.exists() or snapshot_path.exists():
        raise FileExistsError('Use a new evidence directory instead of overwriting this review')
    collection = read(COLLECTION)
    baseline = read(REPO/'artifacts/validation/2026-10-05/g0_baseline/sha256_manifest.json')
    preserved = [row for row in baseline['files'] if row['path'].split('/')[0] in ('src', 'scripts', 'tests', 'config')]
    assert all(digest(REPO/row['path']) == row['sha256'] for row in preserved)
    paths = list(collection['source_sha256'])
    changed = [name for name in paths if digest(REPO/name) != collection['source_sha256'][name]]
    assert set(changed) == {'scripts/validate_multi_vehicle_g0.py', 'tests/test_multi_vehicle_g0.py'}, changed
    tree = ast.parse((REPO/'scripts/validate_multi_vehicle_g0.py').read_text(encoding='utf-8'))
    names = {'visibility_verdict', 'require_horizontal_scene'}
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    assert len(functions) == len(names)
    namespace = {'math': math}
    exec(compile(ast.Module(body=functions, type_ignores=[]), '<current pure predicates>', 'exec'), namespace)
    suite = read(REPO/'config/tests/multi_vehicle_g0.json')
    namespace['require_horizontal_scene'](suite['lidar'])
    rows = []
    for item in collection['cases']:
        original_path = REPO/item['report']
        assert digest(original_path) == item['report_sha256']
        original = read(original_path)
        artifact = original['raw_artifacts']['sequential_scans.jsonl']
        scans_path = REPO/artifact['relative_path']
        assert scans_path.stat().st_size == artifact['bytes']
        assert digest(scans_path) == artifact['sha256']
        with scans_path.open(encoding='utf-8') as stream:
            scans = [json.loads(line) for line in stream if line.strip()]
        active = [scan for scan in scans if scan['source_first_s'] >= original['start_sim_s']
                  and scan['source_last_s'] <= original['start_sim_s']+original['duration_s']]
        assert len(active) == original['active_scan_count'] and active
        decisions = [namespace['visibility_verdict'](scan, original['configuration']['expected']) for scan in active]
        assert all(decision['passed'] for decision in decisions)
        rows.append({'case': item['case'], 'original_report': item['report'],
                     'original_report_sha256': item['report_sha256'],
                     'scan_artifact': artifact['relative_path'], 'scan_sha256': artifact['sha256'],
                     'active_scans': len(active), 'new_visibility_predicate_passed': True,
                     'unknown_rays_total': sum(scan['unknown_rays'] for scan in active),
                     'return_rays_min': min(scan['return_rays'] for scan in active)})
    regression = (OUT/'green.log').read_text(encoding='utf-8')
    assert '260 passed' in regression
    paths += [str(Path(__file__).relative_to(REPO)).replace('\\', '/'),
              str((OUT/'run_tests.sh').relative_to(REPO)).replace('\\', '/')]
    with zipfile.ZipFile(snapshot_path, 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in paths:
            archive.write(REPO/name, name)
    result = {'scope': 'new predicate applied to preserved observations; not a new Gazebo run',
              'original_collection': str(COLLECTION.relative_to(REPO)).replace('\\', '/'),
              'original_collection_sha256': digest(COLLECTION), 'cases': rows,
              'reassessment_passed': True, 'regression_passed': 260,
              'pre_fix_new_tests': {'failed': 21, 'deselected': 22,
                                    'note': '21 test instances, not 21 distinct defects'},
              'new_revision_gazebo_run': False, 'production_files_unchanged': len(preserved),
              'changed_since_original_collection': changed,
              'source_sha256': {name: digest(REPO/name) for name in paths},
              'source_snapshot_sha256': digest(snapshot_path),
              'test_log_sha256': {name: digest(OUT/name) for name in ('red.log', 'green.log')}}
    with report_path.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps({'cases_reassessed': len(rows), 'production_files_unchanged': len(preserved),
                      'regression_passed': 260, 'new_gazebo_run': False}))


if __name__ == '__main__':
    main()

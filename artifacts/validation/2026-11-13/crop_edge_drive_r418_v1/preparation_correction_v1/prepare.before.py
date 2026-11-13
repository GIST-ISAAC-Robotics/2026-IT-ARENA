"""r406 원본 대비 파일별 검토와 r417 이관 뒤에만 동결. ROS/빌드/정책 실행 없음."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
BASE = A.parent/'final_runtime_drive_r406_v1'
R416 = A.parent/'runtime_installed_profile_r416_v1'
SIM = 'src/arena_bringup/launch/simulation.launch.py'
SOURCE_SUFFIXES = {'.py', '.yaml', '.xml', '.xacro', '.sdf', '.cpp', '.hpp', '.stl', '.so'}
FACILITY_INPUTS = [
    'artifacts/validation/2026-11-13/old_signal_610_r326_v1/fixture.json',
    'artifacts/validation/2026-11-13/old_signal_610_r326_v1/support_frame_m.stl',
    'artifacts/validation/2026-11-13/old_signal_610_r326_v1/supports_trial.sdf',
    'artifacts/validation/2026-11-13/old610_common_corner_r333_v1/run.py',
    'artifacts/validation/2026-11-12/signal_post_revision_r291_v1/prepare_fixture.py',
    'artifacts/validation/2026-11-09/signal_post_source_v1/analyze_stl.py',
    'config/tests/c1_standard10_native_angles_v1.json',
]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def write_new(path, data):
    with path.open('x', encoding='utf8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def arguments():
    args = list(read(BASE/'registered.json')['args'])
    replacements = {'--config': A/'config.json', '--actor-overlay': A/'actors.json', '--output': A/'run1'}
    for flag, path in replacements.items():
        args[args.index(flag)+1] = path.relative_to(ROOT).as_posix()
    return args


def common_sources():
    files = {p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in SOURCE_SUFFIXES}
    files.update((ROOT/'scripts').glob('*.py'))
    files.update((ROOT/'tests').rglob('*.py'))
    # 병렬 r416의 firmware-only 6파일도 계보에 드러낸다. 시뮬 선택으로 사용하지 않는다.
    files.update(ROOT/row['path'] for row in read(R416/'integration.json')['files'])
    return {p.relative_to(ROOT).as_posix(): sha(p) for p in sorted(files)}


def source_delta(current):
    old = read(BASE/'registered.json')['input_sha256']
    old = {p: h for p,h in old.items() if p.startswith(('src/', 'scripts/', 'tests/'))}
    return [dict(path=p, before_sha256=old.get(p), after_sha256=current.get(p),
                 kind='changed' if p in old and p in current else ('added_to_snapshot' if p in current else 'missing'))
            for p in sorted(old.keys() | current.keys()) if old.get(p) != current.get(p)]


def draft_review():
    current = common_sources()
    rows = source_delta(current)
    return dict(status='review_required', baseline=str(BASE.relative_to(ROOT)),
                baseline_archive_sha256=sha(BASE/'sources.zip'), changes=rows,
                note='추가 snapshot 파일은 원 기록에 없었다는 뜻이며 모두 이번에 새로 작성됐다는 증거가 아니다.')


def static_contract():
    config, old = read(A/'config.json'), read(BASE/'config.json')
    adjusted = json.loads(json.dumps(config))
    adjusted['scope'] = old['scope']; adjusted['launch']['entry_g1'] = old['launch']['entry_g1']
    assert adjusted == old and config['cases'][0]['duration_s'] == 80.
    assert (A/'actors.json').read_bytes() == (BASE/'actors.json').read_bytes()
    assert '--pilot-marker-z' not in arguments()
    old_header = read(BASE/'expected_header.json')
    expected = read(A/'expected_header.json')
    adjusted_header = json.loads(json.dumps(expected))
    assert adjusted_header['follow_config'].pop('crop_edge_component_trial') is True
    assert adjusted_header == old_header, 'only new ON field may change expected header'
    # STOP 본문은 소유 argv/노드 식별자 두 곳만 바꾼다. 성공/실패 기준은 유지한다.
    stop = (A/'stop_owned.py').read_text(encoding='utf8')
    stop = stop.replace('crop_edge_drive_r418_v1/run.py', 'final_runtime_drive_r406_v1/run.py')
    stop = stop.replace("Node('r418_owned_stop')", "Node('r406_owned_stop')")
    assert stop == (BASE/'stop_owned.py').read_text(encoding='utf8')
    assert (A/'facility_geometry.py').read_bytes() == (BASE/'facility_geometry.py').read_bytes()
    for path in A.rglob('*.py'):
        ast.parse(path.read_text(encoding='utf8'), filename=str(path))
    return dict(config_equal_except_descriptions=True, actors_byte_equal=True,
                owned_stop_body_preserved=True, only_new_selection='g1_crop_edge_component_trial=true',
                selected_yaml_marker_m=.090, expected_mask_count=4,
                thresholds_changed=False, simulator_executed=False)


def integration_files(receipt):
    files = receipt['files']
    return files if isinstance(files, dict) else {row['path']: row['after_sha256'] for row in files}


def freeze(review_path, integration):
    assert not any((A/n).exists() for n in ('prepared.json', 'sources.zip', 'registered.json', 'run1'))
    static = static_contract()
    assert integration is not None and integration.is_file(), 'root r417 validation/integration receipt required'
    receipt = read(integration)
    installed = integration_files(receipt)
    assert installed and all(sha(ROOT/p) == h for p,h in installed.items())
    # r417 영수증을 다른 이관 영수증으로 바꿔 대체하지 않는다.
    assert any('crop_edge' in path for path in installed), 'r417 crop-edge source missing'
    assert 'g1_crop_edge_component_trial' in (ROOT/SIM).read_text(encoding='utf8')
    assert 'crop_edge_component_trial' in (ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py').read_text(encoding='utf8')
    parallel = read(R416/'integration.json')
    assert all(sha(ROOT/p) == h for p,h in integration_files(parallel).items())
    current = common_sources()
    expected = source_delta(current)
    review = read(review_path)
    rows = review['changes']
    bare = [{k: row[k] for k in ('path', 'before_sha256', 'after_sha256', 'kind')} for row in rows]
    assert bare == expected, 'source delta changed since explicit review; preserve and create new review'
    assert review.get('status') == 'reviewed' and all(row.get('reason') for row in rows)
    frozen = {ROOT/name for name in current}
    frozen.update(ROOT/name for name in FACILITY_INPUTS)
    frozen.add(A.parent/'runtime_config_audit_r409_v1/audit_saved.py')
    frozen.update(BASE/name for name in ('config.json', 'actors.json', 'registered.json'))
    frozen.update(BASE/name for name in ('prepared.json', 'expected_header.json', 'run.py', 'stop_owned.py',
                                      'facility_geometry.py', 'launch_adapter.py'))
    frozen.add(R416/'integration.json')
    frozen.add(integration); frozen.add(review_path.resolve())
    frozen.update(p for p in A.rglob('*') if p.is_file() and p.suffix in ('.py', '.json', '.md', '.patch'))
    hashes = {p.relative_to(ROOT).as_posix(): sha(p) for p in sorted(frozen)}
    with zipfile.ZipFile(A/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in hashes:
            archive.write(ROOT/name, name)
    value = dict(**static, args=arguments(), frozen_inputs=hashes,
        source_archive_sha256=sha(A/'sources.zip'), reviewed_source_delta=rows,
        r417_integration_receipt=integration.resolve().relative_to(ROOT).as_posix(),
        r417_precondition='root reviewed common selection/guard/trace validation and integrated before freeze',
        runtime_policy='cad_runtime_r377_v1', current_corner_budget='r396_v1',
        crop_edge_component_trial=True,
        r406_source_archive_sha256=sha(BASE/'sources.zip'),
        policy_dynamic_artifact_imports=False, real_time_factor=.25, one_execution_only=True)
    write_new(A/'prepared.json', value)
    return dict(frozen=True, files=len(hashes), prepared_sha256=sha(A/'prepared.json'),
                source_archive_sha256=value['source_archive_sha256'], simulation_started=False)


def verify_prepared():
    value = read(A/'prepared.json')
    changed = [p for p,h in value['frozen_inputs'].items() if not (ROOT/p).is_file() or sha(ROOT/p) != h]
    assert not changed, ('frozen inputs changed', changed)
    assert sha(A/'sources.zip') == value['source_archive_sha256']
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--inventory', type=Path)
    choice.add_argument('--freeze', type=Path, metavar='REVIEW_JSON')
    choice.add_argument('--check', action='store_true')
    parser.add_argument('--integration', type=Path, help='root가 검증 뒤 작성한 r417 공통 이관 영수증')
    args = parser.parse_args()
    if args.inventory:
        assert args.inventory.resolve().is_relative_to(A)
        value = draft_review()
        write_new(args.inventory, value)
        print(json.dumps(dict(changes=len(value['changes']), **static_contract())))
    elif args.freeze:
        print(json.dumps(freeze(args.freeze, args.integration)))
    else:
        value = verify_prepared()
        print(json.dumps(dict(checked=True, files=len(value['frozen_inputs']), simulation_started=False)))


if __name__ == '__main__':
    main()

"""Windows 파일 준비만 수행한다. r546·공통·r553 실행 입력은 수정하지 않는다."""
from pathlib import Path
import ast
import copy
import hashlib
import json

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = A.parent / 'sparse_actor_measured_steering_r546_v1'
SOURCE = A.parent / 'measured_steering_radial_lap_r553_v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_bytes())

def write(path, value):
    with path.open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')

def once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)

def main():
    assert not (A / 'config.json').exists(), 'preserve existing recipe'
    originals = {p.relative_to(OLD).as_posix(): sha(p) for p in OLD.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    excluded = {'README.md', 'check_recipe.py', 'make_recipe.py', 'preparation_receipt.json',
                'recipe.json', 'source_selection.json', 'recipe_provenance.json', 'config.json',
                'expected_config.json', 'arguments.json'}
    records = []
    for relative in originals:
        if relative in excluded or relative.endswith('.pyc'):
            continue
        src, dst = OLD / relative, A / relative
        assert src.suffix in ('.py', '.json', '.sdf', '.png', '.sh', '.bash') or src.name == '.gitignore', relative
        dst.parent.mkdir(parents=True, exist_ok=True)
        assert not dst.exists(), dst
        data = src.read_bytes()
        if src.suffix in ('.py', '.sh', '.bash'):
            text = data.decode('utf-8').replace(OLD.name, A.name)
            text = text.replace('r546_owned_stop', 'r555_owned_stop').replace("printf 'r546 ", "printf 'r555 ")
            if relative == 'prepare.py':
                text = once(text, '    common={p:h for p,h in old[\'frozen_inputs\'].items()',
                    "    # 장면은 r539, 제어 소스는 root가 확정한 r553 동결본에서 재사용한다.\n"
                    "    source_base=ROOT/source['source_baseline']\n"
                    "    baseline=read(source_base/'prepared.json')\n"
                    "    assert sha(source_base/'sources.zip')==baseline['source_archive_sha256']\n"
                    "    common={p:h for p,h in baseline['frozen_inputs'].items()")
                text = once(text, '    info,selected=verify_recipe()',
                    "    info,selected=verify_recipe()\n    source_base=ROOT/read(A/'source_selection.json')['source_baseline']")
                text = once(text, "A/'preflight/result.json',OLD/'prepared.json',OLD/'sources.zip']:",
                    "A/'preflight/result.json',OLD/'prepared.json',OLD/'sources.zip',source_base/'prepared.json',source_base/'sources.zip']:")
                text = once(text, "*A.glob('*.sdf'),*A.glob('*.md')", "*A.glob('*.sdf'),*A.glob('*.png'),*A.glob('*.md')")
                text = once(text, '"""r520 기준에 root 최종 승인 후보와 r530 자원 경로만 반영한다. 승인 전 동결 금지."""',
                    '"""r539 장면과 r546 sparse actor를 유지하고 root가 선택한 r553 소스를 동결한다."""')
            data = text.encode('utf-8')
        dst.write_bytes(data)
        records.append(dict(path=relative, source=src.relative_to(ROOT).as_posix(),
            before_sha256=originals[relative], after_sha256=sha(dst), byte_exact=data == src.read_bytes()))

    cfg = read(OLD / 'config.json')
    cfg['registered_at'] = '2026-11-14 Windows preparation only; r553 final source/root freeze/run selection pending'
    cfg['scope'] = 'r555 sparse marked actor OFF only; same r546/r539 12sim s scene and original verdicts; reuse r553 final source after root selection'
    cfg['launch']['entry_g1'] = cfg['launch']['entry_g1'].replace(OLD.name, A.name)
    for name in ('config.json', 'expected_config.json'):
        write(A / name, cfg)
    write(A / 'arguments.json', [x.replace(OLD.name, A.name) for x in read(OLD / 'arguments.json')])

    recipe = read(OLD / 'recipe.json')
    recipe.update(status='Windows_ready_not_frozen_waiting_r553_root_selection',
        recipe_predecessor=OLD.relative_to(ROOT).as_posix(),
        intended_source_baseline=SOURCE.relative_to(ROOT).as_posix(),
        actual_source_selection_pending=True, one_execution_only=True, repeat_on_failure=False,
        status_detail='full', WSL_used=False, render_detection_runs=0, dynamic_drive_runs=0)
    recipe['marker']['white_border_each_side_m'] = .0076
    recipe['marker']['on_requires_separate_root_approval'] = True
    write(A / 'recipe.json', recipe)

    selection = dict(control_changes=[], noncontrol_changes=[], additional_sources=[], additional_tests=[],
        source_baseline=SOURCE.relative_to(ROOT).as_posix(),
        scenario_baseline=recipe['base'],
        freeze_authorized=False, wsl_exclusive_owner=False, drive_authorized=False,
        final_production_source_selection_pending=True,
        included_integrations=[], pending_integrations=['root confirms final r553 frozen_inputs and outcome before selecting the traffic run'],
        excluded_integrations=['Jetson/ESP32/physical devices', 'marker assistance ON run'],
        status='Windows recipe only; final r553 source reuse intended, no source selection or execution authorization',
        root_authorization='root/runner own final source selection, WSL resource, freeze and one execution',
        status_detail='full', field_geometry='field_geometry_audit.json',
        reviewed_noncontrol_hashes={}, additional_source_hashes={}, additional_test_hashes={},
        integration_evidence_sha256={}, expected_control_change_paths=[])
    actor_dependency = 'scripts/experimental_facilities.py'
    assert actor_dependency not in read(SOURCE / 'prepared.json')['frozen_inputs']
    selection['additional_sources'] = [actor_dependency]
    selection['additional_source_hashes'] = {actor_dependency: sha(ROOT / actor_dependency)}
    write(A / 'source_selection.json', selection)
    observed = read(SOURCE / 'prepared.json')
    write(A / 'source_baseline_observation.json', dict(
        source_baseline=SOURCE.relative_to(ROOT).as_posix(),
        observed_prepared_sha256=sha(SOURCE / 'prepared.json'),
        archive_sha256_as_recorded=observed['source_archive_sha256'],
        frozen_inputs_count=len(observed['frozen_inputs']),
        adopted=False, archive_rehashed=False, final_selection_owner='root',
        note='현재 r553 준비 영수증을 참고만 했다. 최종 소스 선택/현재 주행 결과/새 freeze 증명이 아니다.'))

    provenance = read(OLD / 'recipe_provenance.json')
    provenance.update(source='r546 original preparation preserved; same r539 scene + r537 sparse marked actor; r553 final source reuse intended',
        files=records, predecessor_files_sha256=originals,
        geometry_generated=False, full_geometry_audit_repeated=False,
        acceptance_unchanged=True, one_execution_only=True, repeat_on_failure=False,
        actual_simulation_started=False)
    write(A / 'recipe_provenance.json', provenance)
    assert {p: sha(OLD / p) for p in originals} == originals
    for path in A.rglob('*.py'):
        ast.parse(path.read_bytes())
    print(json.dumps(dict(copied=len(records), predecessor_files_preserved=len(originals),
        source_selection_pending=True, frozen=False, dynamic_drive_runs=0)))

if __name__ == '__main__':
    main()

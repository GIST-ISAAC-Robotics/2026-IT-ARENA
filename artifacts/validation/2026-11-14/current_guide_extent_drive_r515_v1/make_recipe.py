"""r513의 같은 대표 사례 도구를 r515로 복사한다. 소스 동결/WSL/주행 없음."""
import ast
import hashlib
import json
from pathlib import Path
import shutil

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = A.parent / 'continuous_pass_preview_drive_r513_v1'
OLDER = 'moving_pass_facility_drive_r503_v1'
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
CONTROL = 'src/arena_autonomy/arena_autonomy/moving_pass.py'
TEST = 'tests/test_moving_path_preview.py'
EXACT = ['expected_header.json', 'facilities_only.sdf', 'facility_geometry.py', 'goal_eval.py',
         'geometry_right_side.json', 'course.launch.py', 'launch_adapter.py',
         'launch/demo.launch.py', 'launch/local_pursuit.launch.py', 'launch/simulation.launch.py',
         'execution_v1/extract_failure.py', '.gitignore']
RENAMED = ['run.py', 'preflight.py', 'collect_result.py', 'assess.py', 'stop_owned.py', 'prepare_once.sh',
           'check_recipe.py', 'prepare.py', 'execution_v1/execute_once.sh', 'execution_v1/stop_once.sh',
           'execution_v1/observe.py', 'execution_v1/check_owned_processes.py',
           'execution_v1/finish_collection.bash']


def save(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8', newline='\n') as f:
        f.write(text)


def rename(text):
    return text.replace(OLD.name, A.name).replace('r513', 'r515').replace('R513', 'R515')


def main():
    assert not (A / 'recipe_provenance.json').exists()
    old_prepared = read(OLD / 'prepared.json')
    assert sha(OLD / 'sources.zip') == old_prepared['source_archive_sha256']
    copied = []
    for name in EXACT + RENAMED:
        target = A / name
        target.parent.mkdir(parents=True, exist_ok=True)
        if name in EXACT:
            assert not target.exists()
            shutil.copyfile(OLD / name, target)
        else:
            content = rename((OLD / name).read_text(encoding='utf-8'))
            if name in ('prepare.py', 'check_recipe.py'):
                content = content.replace(OLDER, OLD.name).replace('r503', 'r513')
                if name == 'check_recipe.py':
                    content = content.replace("set(native_changed) <= set(allowed)",
                                              "set(native_changed) <= set(allowed)")
            save(target, content)
        copied.append(dict(path=name, source=(OLD / name).relative_to(ROOT).as_posix(),
                           before_sha256=sha(OLD / name), after_sha256=sha(target), byte_exact=name in EXACT))
    cfg = read(OLD / 'config.json')
    cfg['scope'] = ('r515 same r513 right_side_start 12 sim s with r514 current-forward-guide coverage; '
                    'current CAD/three facilities, visible220mm actor; raw and supplemental verdicts separated')
    cfg['registered_at'] = '2026-11-14 r515 recipe preparation, not execution registration'
    cfg['launch']['entry_g1'] = (A / 'course.launch.py').relative_to(ROOT).as_posix()
    for name in ('config.json', 'expected_config.json'):
        save(A / name, json.dumps(cfg, ensure_ascii=False, indent=2) + '\n')
    args = [rename(x) for x in read(OLD / 'arguments.json')]
    assert args[args.index('--case') + 1] == 'right_side_start'
    save(A / 'arguments.json', json.dumps(args, indent=2) + '\n')
    selection = dict(
        control_changes=[dict(path=CONTROL, before_sha256=old_prepared['frozen_inputs'][CONTROL],
                              after_sha256=None, role='r514 current-forward-guide coverage (root candidate pending final SHA)')],
        noncontrol_changes=[TEST], additional_sources=[], additional_tests=[],
        freeze_authorized=False, wsl_exclusive_owner=False, drive_authorized=False,
        included_integrations=[*read(OLD / 'source_selection.json')['included_integrations'],
                               'r514 current-forward-guide coverage candidate (await root final SHA and tests)'],
        excluded_integrations=['Jetson/firmware/sensor untouched', 'grass priority not mixed into this test'],
        noncontrol_notes={TEST: 'focused candidate regression file; not imported by sim launch'},
        status='Windows recipe prepared; root final source SHA, test evidence and exclusive WSL handover pending',
        root_authorization='2026-11-14 root assignment: Windows preparation only; one run after explicit handover')
    save(A / 'source_selection.json', json.dumps(selection, ensure_ascii=False, indent=2) + '\n')
    names = ('RESULT.md', 'PLAN.md', 'execution_summary.json', 'assessment_summary.json',
             'recipe_provenance.json', 'root_execution_review.json', 'integrity.json',
             'failure_inputs/summary.json', 'preparation_receipt.json', 'source_selection.json')
    original = dict(read(OLD / 'recipe_provenance.json')['original_evidence'])
    original.update({(OLD / n).relative_to(ROOT).as_posix(): sha(OLD / n) for n in names})
    provenance = dict(source='r513 execution recipe; same right_side_start conditions as r499/r503', files=copied,
        original_evidence=original, config_changed_keys_vs_r513=['scope', 'registered_at', 'launch.entry_g1'],
        case_actor_acceptance_unchanged=all(cfg[k] == read(OLD / 'config.json')[k]
                                           for k in ('actor', 'cases', 'acceptance', 'matrix', 'case_source')),
        geometry_reused=dict(file='geometry_right_side.json', r513_sha256=sha(OLD / 'geometry_right_side.json'),
                             reason='same facilities, actor, case and track; retain existing 601 actor samples'),
        threshold_changes=False, current_common_frozen=False, actual_simulation_started=False,
        one_execution_only=True, repeat_on_failure=False)
    save(A / 'recipe_provenance.json', json.dumps(provenance, ensure_ascii=False, indent=2) + '\n')
    for p in A.rglob('*.py'):
        ast.parse(p.read_bytes())
    for p in [*A.rglob('*.sh'), *A.rglob('*.bash')]:
        assert b'\r' not in p.read_bytes(), p
    assert not any((A / n).exists() for n in ('prepared.json', 'sources.zip', 'registered.json', 'run1'))
    print(json.dumps(dict(recipe_files=len(copied), case_actor_acceptance_unchanged=True,
                         frozen=False, WSL_used=False, simulation_started=False)))


if __name__ == '__main__':
    main()

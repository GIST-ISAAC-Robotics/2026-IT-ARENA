"""r503 실행/회수 도구를 같은 조건 그대로 r513에 연결한다. 동결·주행 없음."""
import ast
import hashlib
import json
from pathlib import Path
import shutil

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = A.parent / 'moving_pass_facility_drive_r503_v1'
OLD_NAME = OLD.name
R499 = 'rear_overtake_current_geometry_r499_v1'
R512 = A.parent / 'continuous_pass_preview_r512_v1'
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

CONTROL = ['src/arena_autonomy/arena_autonomy/moving_pass.py']
ROLE = {CONTROL[0]: 'r512 continuous same-path 24cm preview from lateral transition (GPT design)'}
# r503 동결에 이미 들어간 시험 파일이므로 추가가 아닌 비제어 변경으로 기록한다.
NONCONTROL = ['tests/test_moving_path_preview.py']
EXACT = ['expected_header.json', 'facilities_only.sdf', 'facility_geometry.py', 'goal_eval.py',
         'geometry_right_side.json', 'course.launch.py', 'launch_adapter.py',
         'launch/demo.launch.py', 'launch/local_pursuit.launch.py', 'launch/simulation.launch.py',
         'execution_v1/extract_failure.py']
RENAMED = ['run.py', 'preflight.py', 'collect_result.py', 'assess.py', 'stop_owned.py', 'prepare_once.sh',
           'check_recipe.py', 'execution_v1/execute_once.sh', 'execution_v1/stop_once.sh', 'execution_v1/observe.py',
           'execution_v1/check_owned_processes.py', 'execution_v1/finish_collection.bash',
           'execution_v1/attribute_occupied.py', 'execution_v1/attribute_once.sh']


def save(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8', newline='\n') as f:
        f.write(text)


def renamed(text):
    return text.replace(OLD_NAME, A.name).replace('r503', 'r513').replace('R503', 'R513')


def main():
    assert not (A / 'recipe_provenance.json').exists()
    old_prepared = read(OLD / 'prepared.json')
    assert sha(OLD / 'sources.zip') == old_prepared['source_archive_sha256']
    copied = []
    for name in [*EXACT, *RENAMED]:
        target = A / name
        target.parent.mkdir(parents=True, exist_ok=True)
        if name in EXACT:
            assert not target.exists()
            shutil.copyfile(OLD / name, target)
        else:
            text = renamed((OLD / name).read_text(encoding='utf-8'))
            if name == 'check_recipe.py':
                text = text.replace(f"OLD = A.parent / '{R499}'", f"OLD = A.parent / '{OLD_NAME}'")
                text = text.replace('r499', 'r503')
            save(target, text)
        copied.append(dict(path=name, source=(OLD / name).relative_to(ROOT).as_posix(),
                           before_sha256=sha(OLD / name), after_sha256=sha(target), byte_exact=name in EXACT))
    cfg = read(OLD / 'config.json')
    cfg['scope'] = ('r513 same r499/r503 right_side_start 12 sim s with r512 continuous same-path 24cm preview; '
                    'current CAD/three facilities, visible220mm actor; raw and existing supplemental verdicts separated')
    cfg['registered_at'] = '2026-11-14 r513 recipe preparation, not execution registration'
    cfg['launch']['entry_g1'] = (A / 'course.launch.py').relative_to(ROOT).as_posix()
    save(A / 'config.json', json.dumps(cfg, ensure_ascii=False, indent=2) + '\n')
    save(A / 'expected_config.json', json.dumps(cfg, ensure_ascii=False, indent=2) + '\n')
    args = [renamed(x) for x in read(OLD / 'arguments.json')]
    assert args[args.index('--case') + 1] == 'right_side_start'
    save(A / 'arguments.json', json.dumps(args, indent=2) + '\n')

    text = renamed((OLD / 'prepare.py').read_text(encoding='utf-8'))
    # r503 prepare의 기준(r499)을 r503으로 한 단계 옮긴다.
    text = text.replace(R499, OLD_NAME)
    text = text.replace('"""r499 입력과 root 승인 control 4파일만 바꾼', '"""r503 입력과 root 승인 control 1파일만 바꾼')
    save(A / 'prepare.py', text)

    selection = dict(
        control_changes=[dict(path=p, before_sha256=old_prepared['frozen_inputs'][p], after_sha256=sha(ROOT / p),
                              role=ROLE[p]) for p in CONTROL],
        noncontrol_changes=NONCONTROL, additional_sources=[], additional_tests=[],
        freeze_authorized=False, wsl_exclusive_owner=True, drive_authorized=False,
        included_integrations=[*read(OLD / 'source_selection.json')['included_integrations'][:-2],
                               'r498 ThreatViewTransfer output-equivalent optimization (not a decision change; unchanged since r503)',
                               'r512 moving_pass continuous same-path 24cm preview (control candidate; replaces r502 brake-only branch)'],
        excluded_integrations=['r500/r507 install unchanged', 'Jetson/firmware/sensor untouched',
                               'grass priority work not mixed into this test'],
        noncontrol_notes={NONCONTROL[0]: 'r512 focused test file (11 new); not imported by sim launch'},
        status='Recipe ready; freeze waits for root review',
        root_authorization='2026-11-14 root assignment: prepare r513 once with r499/r503 conditions; execute once only after root review')
    save(A / 'source_selection.json', json.dumps(selection, ensure_ascii=False, indent=2) + '\n')

    evidence = [OLD / n for n in ('RESULT.md', 'PLAN.md', 'execution_summary.json', 'assessment_summary.json',
                                  'recipe_provenance.json', 'root_execution_review.json', 'integrity.json',
                                  'failure_inputs/summary.json', 'failure_inputs/occupied_attribution.json')]
    evidence += [R512 / n for n in ('RESULT.md', 'validation.json', 'candidate_replay.json',
                                    'moving_pass.before.py', 'test_moving_path_preview.before.py')]
    original = {p.relative_to(ROOT).as_posix(): sha(p) for p in evidence}
    # r503이 근거로 둔 r499·r502·r498·원 사례 자료도 그대로 이어서 대조한다.
    original.update(read(OLD / 'recipe_provenance.json')['original_evidence'])
    assert sha(R512 / 'moving_pass.before.py') == old_prepared['frozen_inputs'][CONTROL[0]]
    assert sha(R512 / 'test_moving_path_preview.before.py') == old_prepared['frozen_inputs'][NONCONTROL[0]]
    assert sha(ROOT / CONTROL[0]) == read(R512 / 'validation.json')['moving_pass_sha256']
    provenance = dict(source='r503 execution recipe (same right_side_start conditions as r499)', files=copied,
        original_evidence=original,
        config_changed_keys_vs_r503=['scope', 'registered_at', 'launch.entry_g1'],
        case_actor_acceptance_unchanged=all(cfg[k] == read(OLD / 'config.json')[k]
                                            for k in ('actor', 'cases', 'acceptance', 'matrix', 'case_source')),
        geometry_reused=dict(file='geometry_right_side.json', r503_sha256=sha(OLD / 'geometry_right_side.json'),
                             reason='same facilities_only.sdf, actor config, case and track as r499/r503; 601 actor samples'),
        r512_before_copies_match_r503_frozen=True, current_moving_pass_matches_r512_validation=True,
        threshold_changes=False, current_common_frozen=False, actual_simulation_started=False,
        one_execution_only=True, repeat_on_failure=False)
    save(A / 'recipe_provenance.json', json.dumps(provenance, ensure_ascii=False, indent=2) + '\n')
    for p in A.rglob('*.py'):
        ast.parse(p.read_bytes())
    for p in [*A.rglob('*.sh'), *A.rglob('*.bash')]:
        assert b'\r' not in p.read_bytes(), p
    assert not any((A / n).exists() for n in ('prepared.json', 'sources.zip', 'registered.json', 'run1'))
    print(json.dumps(dict(recipe_copied=len(copied), control=CONTROL, noncontrol=NONCONTROL,
                          case_actor_acceptance_unchanged=provenance['case_actor_acceptance_unchanged'])))


if __name__ == '__main__':
    main()

"""r499 실행/회수 도구를 같은 조건 그대로 r503에 연결한다. 동결·주행 없음."""
import ast
import hashlib
import json
from pathlib import Path
import shutil

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = A.parent / 'rear_overtake_current_geometry_r499_v1'
OLD_NAME = OLD.name
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

CONTROL = ['src/arena_autonomy/arena_autonomy/moving_pass.py',
           'src/arena_autonomy/arena_autonomy/local_follow.py',
           'src/arena_autonomy/arena_autonomy/traffic_solo_gap.py',
           'src/arena_autonomy/arena_autonomy/pass_release_trial.py']
ROLE = {CONTROL[0]: 'r502 same observed path 24cm preview candidate (GPT design)',
        CONTROL[1]: 'r498 output-equivalent ThreatViewTransfer optimization (r501 ARM 508/508)',
        CONTROL[2]: 'r498 output-equivalent ThreatViewTransfer optimization (r501 ARM 508/508)',
        CONTROL[3]: 'r498 output-equivalent ThreatViewTransfer optimization (r501 ARM 508/508)'}
NONCONTROL = ['scripts/steering_zero_profile.py', 'src/arena_autonomy/arena_autonomy/race_ops_page.py']
TESTS = ['tests/test_moving_path_preview.py', 'tests/test_threat_view_transfer.py']
EXACT = ['expected_header.json', 'facilities_only.sdf', 'facility_geometry.py', 'goal_eval.py',
         'geometry_right_side.json', 'course.launch.py', 'launch_adapter.py',
         'launch/demo.launch.py', 'launch/local_pursuit.launch.py', 'launch/simulation.launch.py']
RENAMED = ['run.py', 'preflight.py', 'collect_result.py', 'assess.py', 'stop_owned.py', 'prepare_once.sh',
           'execution_v1/execute_once.sh', 'execution_v1/stop_once.sh', 'execution_v1/observe.py',
           'execution_v1/check_owned_processes.py', 'execution_v1/finish_collection.bash']


def save(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8', newline='\n') as f:
        f.write(text)


def renamed(text):
    return text.replace(OLD_NAME, A.name).replace('r499', 'r503').replace('R499', 'R503')


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
            save(target, renamed((OLD / name).read_text(encoding='utf-8')))
        copied.append(dict(path=name, source=(OLD / name).relative_to(ROOT).as_posix(),
                           before_sha256=sha(OLD / name), after_sha256=sha(target), byte_exact=name in EXACT))
    cfg = read(OLD / 'config.json')
    cfg['scope'] = ('r503 same r499 right_side_start 12 sim s with r502 moving-pass 24cm preview candidate '
                    'and r498 output-equivalent optimization; current CAD/three facilities, visible220mm actor; '
                    'raw and existing supplemental verdicts separated')
    cfg['registered_at'] = '2026-11-14 r503 recipe preparation, not execution registration'
    cfg['launch']['entry_g1'] = (A / 'course.launch.py').relative_to(ROOT).as_posix()
    save(A / 'config.json', json.dumps(cfg, ensure_ascii=False, indent=2) + '\n')
    save(A / 'expected_config.json', json.dumps(cfg, ensure_ascii=False, indent=2) + '\n')
    args = [renamed(x) for x in read(OLD / 'arguments.json')]
    assert args[args.index('--case') + 1] == 'right_side_start'
    save(A / 'arguments.json', json.dumps(args, indent=2) + '\n')

    text = renamed((OLD / 'prepare.py').read_text(encoding='utf-8'))
    text = text.replace('traffic_current_geometry_r496_v1', OLD_NAME)
    text = text.replace('"""r496 입력과 기존 오른쪽 옆차 대표를 새 실행으로 동결한다.',
                        '"""r499 입력과 root 승인 control 4파일만 바꾼 같은 오른쪽 옆차 대표를 새 실행으로 동결한다.')
    text = text.replace("role='control_python' if p in allowed else 'unexecuted_jetson_utility'",
                        "role='control_python' if p in allowed else 'noncontrol_not_launched_in_sim'")
    save(A / 'prepare.py', text)

    selection = dict(
        control_changes=[dict(path=p, before_sha256=old_prepared['frozen_inputs'][p], after_sha256=sha(ROOT / p),
                              role=ROLE[p]) for p in CONTROL],
        noncontrol_changes=NONCONTROL, additional_sources=[], additional_tests=TESTS,
        freeze_authorized=False, wsl_exclusive_owner=True, drive_authorized=False,
        included_integrations=[*read(OLD / 'source_selection.json')['included_integrations'],
                               'r502 moving_pass same observed path 24cm preview (control candidate)',
                               'r498 ThreatViewTransfer output-equivalent optimization (not a decision change)'],
        excluded_integrations=['r500 default install unchanged', 'Jetson/firmware/sensor untouched'],
        noncontrol_notes={NONCONTROL[0]: 'servo zero CLI; not imported by sim launch',
                          NONCONTROL[1]: 'operator panel page used only by race_ops_node; race_ops not in g1_follow launch'},
        status='Recipe ready; freeze waits for root review and race_ops_page owner completion',
        root_authorization='2026-11-14 root assignment: prepare r503 once with r499 conditions; execute once only after root review')
    save(A / 'source_selection.json', json.dumps(selection, ensure_ascii=False, indent=2) + '\n')

    evidence = [OLD / n for n in ('RESULT.md', 'PLAN.md', 'execution_summary.json', 'assessment_summary.json',
                                  'geometry.py', 'geometry_right_side.json', 'recipe_provenance.json',
                                  'failure_inputs/summary.json', 'failure_inputs/occupied_attribution.json')]
    r502 = A.parent / 'moving_pass_facility_r502_v1'
    evidence += [r502 / n for n in ('RESULT.md', 'validation.json', 'source_change.json', 'change.patch',
                                    'candidate_replay.json')]
    r498 = A.parent / 'threat_view_reuse_r498_v1'
    evidence += [r498 / 'integration_v1/receipt.json', r498 / 'integration_v1/RESULT.md',
                 A.parent / 'threat_view_reuse_arm_r501_v1/RESULT.md']
    original = {p.relative_to(ROOT).as_posix(): sha(p) for p in evidence}
    # r499가 근거로 둔 원 사례·r496 자료도 그대로 이어서 대조한다.
    original.update(read(OLD / 'recipe_provenance.json')['original_evidence'])
    provenance = dict(source='r499 execution recipe (same right_side_start conditions)', files=copied,
        original_evidence=original,
        config_changed_keys_vs_r499=['scope', 'registered_at', 'launch.entry_g1'],
        case_actor_acceptance_unchanged=all(cfg[k] == read(OLD / 'config.json')[k]
                                            for k in ('actor', 'cases', 'acceptance', 'matrix', 'case_source')),
        geometry_reused=dict(file='geometry_right_side.json', r499_sha256=sha(OLD / 'geometry_right_side.json'),
                             reason='same facilities_only.sdf, actor config, case and track; 601 actor samples'),
        threshold_changes=False, current_common_frozen=False, actual_simulation_started=False,
        one_execution_only=True, repeat_on_failure=False)
    save(A / 'recipe_provenance.json', json.dumps(provenance, ensure_ascii=False, indent=2) + '\n')
    for p in A.rglob('*.py'):
        ast.parse(p.read_bytes())
    for p in [*A.rglob('*.sh'), *A.rglob('*.bash')]:
        assert b'\r' not in p.read_bytes(), p
    assert not any((A / n).exists() for n in ('prepared.json', 'sources.zip', 'registered.json', 'run1'))
    print(json.dumps(dict(recipe_copied=len(copied), control=len(CONTROL), noncontrol=NONCONTROL, tests=TESTS,
                          case_actor_acceptance_unchanged=provenance['case_actor_acceptance_unchanged'])))


if __name__ == '__main__':
    main()

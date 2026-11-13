"""r496 실행/회수 도구와 기존 right_side_start를 연결한다. 동결·주행 없음."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import shutil

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = A.parent / 'traffic_current_geometry_r496_v1'
CASE = ROOT / 'artifacts/validation/2026-11-08/right_side_newcourse_v1'
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def save(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8', newline='\n') as f:
        f.write(text)


def renamed(text):
    return text.replace('traffic_current_geometry_r496_v1', A.name).replace('r496', 'r499').replace('R496', 'R499')


def main():
    assert not (A / 'recipe_provenance.json').exists()
    exact = ['expected_header.json', 'facilities_only.sdf', 'facility_geometry.py']
    copied = []
    names = [*exact, 'course.launch.py', 'launch_adapter.py', 'stop_owned.py',
             'launch/demo.launch.py', 'launch/local_pursuit.launch.py', 'launch/simulation.launch.py',
             'execution_v1/execute_once.sh', 'execution_v1/stop_once.sh', 'execution_v1/observe.py',
             'execution_v1/check_owned_processes.py', 'prepare_once.sh']
    for name in names:
        target = A / name
        target.parent.mkdir(parents=True, exist_ok=True)
        if name in exact:
            assert not target.exists()
            shutil.copyfile(OLD / name, target)
        else:
            save(target, renamed((OLD / name).read_text(encoding='utf-8')))
        copied.append(dict(path=name, source=(OLD / name).relative_to(ROOT).as_posix(),
                           before_sha256=sha(OLD / name), after_sha256=sha(target), byte_exact=name in exact))
    cfg = read(OLD / 'config.json')
    original = read(CASE / 'new_track_course.json')
    cfg['scope'] = 'r499 existing right-side vehicle pass and continued return; not rear overtaking; current CAD/facilities, visible220mm actor; raw and existing supplemental verdicts separated'
    cfg['registered_at'] = '2026-11-14 r499 recipe preparation, not execution registration'
    cfg['launch']['entry_g1'] = (A / 'course.launch.py').relative_to(ROOT).as_posix()
    cfg['actor'] = copy.deepcopy(original['actor'])
    cfg['actor']['height_m'] = .22
    cfg['actor']['visual_outline'] = 'box'
    cfg['actor']['visible_section'] = '기존120mm 상자를 현재 C1 Z134.6mm에서 보이는220mm로 명시한 행동 시험 가정. 낮은 상대 감지/기존120mm 성공 아님.'
    cfg['cases'] = copy.deepcopy(original['cases'])
    cfg['acceptance']['right_side_start'] = copy.deepcopy(original['acceptance']['slow_lead'])
    cfg['matrix'] = {'g1g2': {'cases': ['right_side_start'], 'seeds': [0]}}
    cfg['case_source'] = dict(path=(CASE / 'new_track_course.json').relative_to(ROOT).as_posix(),
                              sha256=sha(CASE / 'new_track_course.json'))
    save(A / 'config.json', json.dumps(cfg, ensure_ascii=False, indent=2) + '\n')
    args = [renamed(x) for x in read(OLD / 'arguments.json')]
    args[args.index('--case') + 1] = 'right_side_start'
    save(A / 'arguments.json', json.dumps(args, indent=2) + '\n')

    text = renamed((OLD / 'run.py').read_text(encoding='utf-8'))
    text = text.replace("case='straight_moving_lead', duration_sim_s=10", "case='right_side_start', duration_sim_s=12")
    save(A / 'run.py', text)
    text = (OLD / 'prepare.py').read_text(encoding='utf-8')
    text = text.replace("OLD=A.parent/'traffic_return_recovery_r490_v1'", "OLD=A.parent/'traffic_current_geometry_r496_v1'")
    text = text.replace('원 r490 입력과 root 승인 delta만 새 실행으로 동결한다.', 'r496 입력과 기존 오른쪽 옆차 대표를 새 실행으로 동결한다.')
    text = text.replace("'traffic_return_recovery_r490_v1','traffic_current_geometry_r496_v1'", "'traffic_current_geometry_r496_v1','rear_overtake_current_geometry_r499_v1'")
    text = text.replace("cfg['scope']=before['scope'];cfg['launch']['entry_g1']=before['launch']['entry_g1'];assert cfg==before", "assert cfg==read(A/'expected_config.json')")
    text = text.replace("assert read(A/'arguments.json')==expected", "expected[expected.index('--case')+1]='right_side_start'\n    assert read(A/'arguments.json')==expected")
    text = text.replace("};assert allowed", "}")
    text = text.replace('max_sim_seconds=10', 'max_sim_seconds=12')
    text = text.replace("assert read(A/'preflight/result.json')['passed']", "assert read(A/'preflight/result.json')['passed']\n    assert read(A/'geometry_right_side.json')['passed']\n    for name, digest in read(A/'recipe_provenance.json')['original_evidence'].items():\n        assert sha(ROOT/name)==digest,name\n        selected[name]=digest")
    save(A / 'prepare.py', text)
    save(A / 'expected_config.json', json.dumps(cfg, ensure_ascii=False, indent=2) + '\n')
    text = renamed((OLD / 'preflight.py').read_text(encoding='utf-8'))
    text = text.replace("info,_=verify_recipe();imports=[]", "info,_=verify_recipe();imports=[]\n    assert read(A/'geometry_right_side.json')['passed']")
    save(A / 'preflight.py', text)
    selection = dict(control_changes=[], noncontrol_changes=[], additional_sources=[], additional_tests=[],
                     freeze_authorized=False, wsl_exclusive_owner=False, drive_authorized=False,
                     included_integrations=read(OLD / 'source_selection.json')['included_integrations'],
                     excluded_integrations=['r498 remains isolated stage'],
                     status='Windows recipe ready; root must review conditions and authorize freeze',
                     root_authorization='candidate preparation only; no common changes or driving')
    save(A / 'source_selection.json', json.dumps(selection, ensure_ascii=False, indent=2) + '\n')
    # 기존 별도 평가의 숫자/상태 조건은 유지하고 현재 차체 치수만 명시 입력으로 바꾼다.
    goal = (CASE / 'goal_eval.py').read_text(encoding='utf-8')
    goal = goal.replace('MINIMUM_GAP_M = .08', 'EGO_LENGTH_M = .207\nEGO_WIDTH_M = .15\nACTOR_LENGTH_M = .20\nMINIMUM_GAP_M = .08')
    goal = goal.replace("rectangle(row['x'], row['y'], row['yaw'], .20, .15)", "rectangle(row['x'], row['y'], row['yaw'], EGO_LENGTH_M, EGO_WIDTH_M)")
    goal = goal.replace('body = a_s-ego_s-.20', 'body = a_s-ego_s-(EGO_LENGTH_M+ACTOR_LENGTH_M)/2')
    goal = goal.replace('ego_s-.10 > a_s+.10', 'ego_s-EGO_LENGTH_M/2 > a_s+ACTOR_LENGTH_M/2')
    goal = goal.replace('(np.abs(ego_s-a_s)<=.20)', '(np.abs(ego_s-a_s)<=(EGO_LENGTH_M+ACTOR_LENGTH_M)/2)')
    save(A / 'goal_eval.py', goal)
    text = renamed((OLD / 'collect_result.py').read_text(encoding='utf-8'))
    text = text.replace('one 10sim-second visible-actor scene', 'one 12sim-second right-side visible-actor scene; not rear overtaking')
    text = text.replace("['straight_moving_lead']", "['right_side_start']")
    save(A / 'collect_result.py', text)
    text = renamed((OLD / 'execution_v1/finish_collection.bash').read_text(encoding='utf-8'))
    text += 'python3 -B "$a/assess.py"\n'
    save(A / 'execution_v1/finish_collection.bash', text)
    evidence = [CASE / name for name in ('new_track_course.json', 'goal_eval.py', 'README.md', 'review_report.json', 'review_goal.json')]
    evidence += [OLD / 'native_v1/receipt.json', OLD / 'execution_summary.json', OLD / 'source_selection.json']
    provenance = dict(source='r496 execution recipe + existing right_side_start case', files=copied,
        original_evidence={p.relative_to(ROOT).as_posix():sha(p) for p in evidence},
        case_exact=cfg['cases']==original['cases'], raw_acceptance_alias_exact=cfg['acceptance']['right_side_start']==original['acceptance']['slow_lead'],
        expected_changed_config_keys=['scope', 'registered_at', 'launch.entry_g1', 'actor', 'cases', 'acceptance.right_side_start', 'matrix', 'case_source'],
        actor_change_vs_original=dict(height_m=[.12,.22], visual_outline='box made explicit, original default box unchanged'),
        actor_command_hz=50, threshold_changes=False, current_common_frozen=False, actual_simulation_started=False,
        one_execution_only=True, repeat_on_failure=False,
        rejected_rear_geometry='geometry.json:25 facility overlap samples; not the selected case',
        selected_geometry='geometry_right_side.json:601 actor path samples, facility/road passed')
    save(A / 'recipe_provenance.json', json.dumps(provenance, ensure_ascii=False, indent=2) + '\n')
    for p in A.rglob('*.py'):
        ast.parse(p.read_bytes())
    for p in A.rglob('*.sh'):
        assert b'\r' not in p.read_bytes(), p
    assert not any((A / n).exists() for n in ('prepared.json', 'sources.zip', 'registered.json', 'run1'))
    print(json.dumps(dict(recipe_copied=len(copied), case='right_side_start', duration_sim_s=12,
                         current_common_frozen=False, actual_simulation_started=False)))


if __name__ == '__main__':
    main()

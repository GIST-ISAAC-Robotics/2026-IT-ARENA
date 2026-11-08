"""저장된 원시 기록에 기존 평가 함수를 다시 적용한다. 새 주행이 아니며 run1/report.json은 덮지 않는다.

원 실행에서 파생 설정이 사례 이름만 바꾸고 기존 `acceptance.five_vehicle_grid` 항목을 새 이름으로 옮기지 않아
`evaluate_run`이 KeyError('minimum_progress_m')로 끝났다(집계 입력 오류). 여기서는 같은 항목을 값 변경 없이
새 사례 이름에도 둔 수정 설정 사본으로 평가만 다시 한다.

사용: reevaluate.py control   r48 실행에 같은 재계산을 적용해 하네스 내부 결과와 일치하는지 확인
      reevaluate.py run1      이 폴더 run1 재평가 → reeval/ 아래에 기록
"""
import copy
import hashlib
import json
from pathlib import Path
import sys

import g1_traffic_eval as ev
import validate_g1_traffic as vt
from audit_local_pursuit_road import road_polygon
from traffic_actors import overlay_case

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
R48 = ROOT/'artifacts/validation/2026-11-08/observed_corner_continuity_r48_v1'
BASE_CASE = 'five_vehicle_grid'
CASE = 'five_vehicle_grid_staggered'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
rows = lambda p: [json.loads(line) for line in Path(p).read_bytes().splitlines() if line.strip()]


def arg(args, name):
    return args[args.index(name)+1] if name in args else None


def evaluate(run, registered, config_path):
    report = json.loads((run/'report.json').read_bytes())
    args = registered['args']
    config = vt.load_config(config_path)
    overlay = arg(args, '--actor-overlay')
    case = overlay_case(vt.case_named(config, arg(args, '--case')), ROOT/overlay if overlay else None)
    centerline, scene = vt.load_centerline(vt.TRACK_WORLDS[arg(args, '--track')])
    s0 = vt.actor_start_s(config, scene, int(arg(args, '--seed')), case)
    t_green = report['green_observed_sim_s']

    def schedule_pose(t):  # validate_g1_traffic.py 하네스 내부 정의와 같다
        elapsed = 0. if t_green is None else t-t_green
        x, y, _ = vt.primary_pose_at(config, centerline, s0, case, elapsed)
        return x, y

    evaluation = ev.evaluate_run(
        config=config, case=case, mode=arg(args, '--mode'), t_green=t_green,
        trace=json.loads((run/'trace.json').read_bytes()), ego=rows(run/'ego_poses.jsonl'),
        actor=rows(run/'actor_poses.jsonl'), contacts=rows(run/'actor_contacts.jsonl'),
        commands=json.loads((run/'actor_commands.json').read_bytes()), stop_start=report.get('stop_request_sim_s'),
        schedule_pose=schedule_pose, centerline=centerline, s0=s0, road=road_polygon(run/'runtime_world.sdf'),
        runtime_world=(run/'runtime_world.sdf').read_text(encoding='utf-8'), contact_evidence=report['contact_evidence'])
    redone = copy.deepcopy(report)
    redone['evaluation'] = evaluation
    ev.finalize_verdict(redone, arg(args, '--mode'))
    if 'actor_overlay' in report:  # 하네스와 같은 순서로 저장된 추가 상대 결과를 합친다
        extra = report['additional_actors']
        redone['harness_valid'] = redone['harness_valid'] and extra['harness_valid']
        redone['run_valid'] = redone['run_valid'] and extra['harness_valid']
        redone['behavior_gates_passed'] = redone['behavior_gates_passed'] and extra['behavior_gates_passed']
        redone['passed'] = redone['run_valid'] and redone['behavior_gates_passed']
    if 'handoff_execution' in report:
        for key in ('harness_valid', 'run_valid', 'passed'):
            redone[key] = bool(redone.get(key)) and report['handoff_execution']['passed']
    return report, redone


def control():
    registered = json.loads((R48/'registered.json').read_bytes())
    original, redone = evaluate(R48/'run1', registered, ROOT/arg(registered['args'], '--config'))
    keys = ('evaluation', 'gates', 'integrity', 'stop_checks', 'harness_valid', 'run_valid', 'behavior_gates_passed', 'passed')
    same = {k: json.dumps(original.get(k), sort_keys=True) == json.dumps(redone.get(k), sort_keys=True) for k in keys}
    out = dict(scope='reevaluation method control on r48 run1 (current sources); read-only', same=same, all_same=all(same.values()),
               r48_report_sha256=sha(R48/'run1/report.json'))
    print(json.dumps(out))
    return out


def main(which):
    if which == 'control':
        out = control()
        (A/'reeval').mkdir(exist_ok=True)
        (A/'reeval/control_r48.json').write_text(json.dumps(out, indent=2)+'\n', encoding='utf-8')
        return 0 if out['all_same'] else 1
    folder = A/'reeval'
    assert json.loads((folder/'control_r48.json').read_bytes())['all_same'] is True
    assert not (folder/'report_reevaluated.json').exists()
    config = json.loads((A/'new_track_course.json').read_bytes())
    assert CASE not in config['acceptance']
    fixed = copy.deepcopy(config)
    fixed['acceptance'][CASE] = copy.deepcopy(config['acceptance'][BASE_CASE])
    fixed['reevaluation_note'] = 'acceptance.five_vehicle_grid copied unchanged to the renamed case; original registered config preserved'
    fixed_path = folder/'new_track_course_acceptance_fixed.json'
    with open(fixed_path, 'x', encoding='utf-8', newline='\n') as f:
        f.write(json.dumps(fixed, ensure_ascii=False, indent=2)+'\n')
    registered = json.loads((A/'registered.json').read_bytes())
    original, redone = evaluate(A/'run1', registered, fixed_path)
    redone['reevaluation'] = dict(new_drive=False, reason="original evaluate_run KeyError: 'minimum_progress_m' (acceptance not copied to renamed case)",
                                  original_report_sha256=sha(A/'run1/report.json'), fixed_config=fixed_path.relative_to(ROOT).as_posix(),
                                  fixed_config_sha256=sha(fixed_path), registered_config_sha256=sha(A/'new_track_course.json'),
                                  original_evaluation=original['evaluation'],
                                  original_verdict={k: original.get(k) for k in ('harness_valid', 'run_valid', 'passed')})
    with open(folder/'report_reevaluated.json', 'x', encoding='utf-8', newline='\n') as f:
        f.write(json.dumps(redone, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: redone.get(k) for k in ('passed', 'harness_valid', 'run_valid', 'behavior_gates_passed', 'gates')}))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))

"""r371 대비 설정/명령/원 소스 차이를 기록한다. ROS/Gazebo를 시작하지 않는다."""
import argparse
import ast
import copy
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
BASE = A.parent/'relative_body_cad_drive_r371_v1'
POLICY = A.parent/'released_current_r375_v1'
ALLOWED_UNRELATED_CHANGES = {
    'scripts/steering_zero_profile.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/steering_calibration.py',
    'tests/test_steering_calibration.py',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def configuration():
    value = json.loads((BASE/'config.json').read_bytes())
    value['scope'] = 'r377: r371 설정/기하/CAD 조향/속도/센서/물성/평가를 유지하고 현재 검사에 r375 RELEASED parallel 상대거리 공백만 명시. r361 계획·두220mm 상대·80초/RTF0.25·원 STOP/접촉/노면/정체 기준 유지. 단1회이며 실차/실시간 증명 아님.'
    value['launch']['entry_g1'] = 'r377 course.launch.py plus unchanged r361 plan; r375 current RELEASED parallel policy'
    return value


def arguments():
    value = list(json.loads((BASE/'registered.json').read_bytes())['args'])
    for flag, path in (('--config', A/'config.json'), ('--actor-overlay', A/'actors.json'),
                       ('--output', A/'run1')):
        value[value.index(flag)+1] = str(path)
    # 原 기록은 WSL 절대 경로다. 실행 ROOT에서 같은 상대 파일로만 이식한다.
    for flag in ('--local-sensor-config', '--vehicle-config'):
        index = value.index(flag)+1
        relative = value[index].split('/IT ARENA local/', 1)[1]
        value[index] = str(ROOT/relative)
    return value


def normalized_args(value):
    return [v.replace('\\', '/').split('/IT ARENA local/')[-1] if 'IT ARENA local' in v else v for v in value]


def verify_prepared():
    data = json.loads((A/'prepared.json').read_bytes())
    changed = [name for name, expected in data['frozen_inputs'].items()
               if not (ROOT/name).is_file() or sha(ROOT/name) != expected]
    assert not changed, ('prepared input changed', changed)
    assert not (A/'registered.json').exists() and not (A/'run1').exists(), 'one execution only'
    return data


def prepare():
    original = json.loads((BASE/'config.json').read_bytes())
    selected = configuration()
    stripped = copy.deepcopy(selected)
    stripped['scope'] = original['scope']
    stripped['launch']['entry_g1'] = original['launch']['entry_g1']
    assert stripped == original, 'runtime configuration differs'
    assert original['cases'][0]['duration_s'] == 80.0
    before = json.loads((BASE/'registered.json').read_bytes())
    old_args, new_args = normalized_args(before['args']), normalized_args(arguments())
    differences = [dict(index=i, before=x, after=y) for i, (x, y) in enumerate(zip(old_args, new_args)) if x != y]
    assert len(old_args) == len(new_args) and len(differences) == 3
    assert {old_args[r['index']-1] for r in differences} == {'--config', '--actor-overlay', '--output'}
    # 바이트 사본으로 경로가 바뀌는 launch와 profile은 실행 ID 외 차이가 없어야 한다.
    unchanged = ('profile.json', 'launch_prefix.py', 'course.launch.py',
                 'launch/demo.launch.py', 'launch/local_pursuit.launch.py', 'launch/simulation.launch.py')
    for name in unchanged:
        old = (BASE/name).read_text(encoding='utf8').replace('r371', 'r377').replace('R371', 'R377')
        assert (A/name).read_text(encoding='utf8') == old, ('launch differs', name)
    for path in A.rglob('*.py'): ast.parse(path.read_bytes(), filename=str(path))
    prior_files = before['input_sha256']
    changed = []
    frozen = {}
    # 원 실행의 소스/정책 계보를 현재 값으로 동결한다. 로그/기계 생성 결과는 제외한다.
    for name, old_sha in prior_files.items():
        path = ROOT/name
        if path.suffix not in ('.py', '.yaml', '.xml', '.xacro', '.sdf', '.cpp', '.hpp', '.stl'): continue
        if '/relative_body_cad_drive_r371_v1/' in name: continue
        assert path.is_file(), ('missing original source', name)
        frozen[name] = sha(path)
        if frozen[name] != old_sha:
            changed.append(dict(path=name, before_sha256=old_sha, after_sha256=frozen[name]))
    assert all(row['path'] in ALLOWED_UNRELATED_CHANGES for row in changed), changed
    for path in [*A.rglob('*.py'), A/'profile.json', *POLICY.glob('*.py'),
                 POLICY/'results_native_v1/comparison.json', BASE/'config.json', BASE/'actors.json', BASE/'registered.json']:
        frozen[path.relative_to(ROOT).as_posix()] = sha(path)
    # archive는 dynamic import의 __module__ 등록 여부와 관계없이 이 파일들을 직접 포함한다.
    report = dict(prepared_only=True, runtime_not_started=True, config_equal_except_descriptions=True,
        argument_differences=differences, original_timeouts=original['timeouts'], duration_s=80.,
        real_time_factor=.25, unchanged_launch_files=list(unchanged), source_delta_since_r371=changed,
        source_delta_scope='조립 후 수동 영점 프로필 생성/직렬화 파일 및 시험만. 새 구동 연결/현재 정책/CAD 모델에는 적용하지 않음.',
        frozen_inputs=frozen, policy_sha256=sha(POLICY/'candidate.py'),
        required_actual_application='current_released_relative_body.applied=true in current_rebase output',
        one_run_only=True, thresholds_changed=False, ground_truth_evaluation_only=True)
    with (A/'prepared.json').open('x', encoding='utf8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'frozen_inputs'}, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if args.check:
        value = verify_prepared()
        print(json.dumps(dict(checked=True, frozen_input_count=len(value['frozen_inputs']), runtime_started=False)))
    else:
        prepare()

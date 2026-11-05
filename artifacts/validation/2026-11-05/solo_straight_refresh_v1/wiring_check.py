"""후보 트리(cand)의 기본 OFF 플래그·launch·검사기·진입점 연결 정적 확인. 실행·Gazebo 없음.

사용(WSL, ROS source 후): PYTHONPATH=<cand>/src/arena_autonomy:... python3 -B wiring_check.py <cand 루트>
"""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
CAND = Path(sys.argv[1]).resolve()
REGISTERED = json.loads((HERE.parent/'solo_jr60p20_baseline_v1/registered.json').read_bytes())


def strip_paths(args):
    # 출력·센서 경로는 트리 위치에 따라 달라지므로 파일 이름만 비교한다.
    return [Path(a).name if a.startswith('/') else a for a in args]


def print_args(*extra):
    out = subprocess.run([sys.executable, '-B', 'scripts/run_current_solo.py',
                          'artifacts/validation/2026-11-05/solo_jr60p20_baseline_v1/run1', '--print-args', *extra],
                         cwd=CAND, check=True, capture_output=True, text=True).stdout
    return json.loads(out)


def main():
    checks = {}
    registered = strip_paths(REGISTERED['entry']['print_args'])
    checks['entry_default_args_equal_run1_registration'] = strip_paths(print_args()) == registered
    checks['entry_straight_budget_adds_only_flag'] = strip_paths(print_args('--straight-budget')) == registered+['--g1-solo-straight-budget']
    help_text = subprocess.run([sys.executable, '-B', 'scripts/validate_basic_autonomy.py', '--help'], cwd=CAND,
                               capture_output=True, text=True).stdout
    checks['validator_accepts_flag'] = '--g1-solo-straight-budget' in help_text
    validator = (CAND/'scripts/validate_basic_autonomy.py').read_text(encoding='utf-8')
    checks['validator_appends_launch_arg_only_with_flag'] = (
        validator.count("extra_arguments.append('g1_solo_straight_budget_trial:=true')") == 1
        and "if args.g1_solo_straight_budget:" in validator)
    spec = importlib.util.spec_from_file_location('g1_follow_launch', CAND/'src/arena_bringup/launch/g1_follow.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    from launch.actions import DeclareLaunchArgument, GroupAction
    found = {}
    for action in description.entities:
        for sub in (action.get_sub_entities() if isinstance(action, GroupAction) else [action]):
            if isinstance(sub, DeclareLaunchArgument):
                found[sub.name] = sub.default_value
    default = found.get('g1_solo_straight_budget_trial')
    checks['launch_declares_flag_default_false'] = default is not None and ''.join(
        getattr(part, 'text', str(part)) for part in default) == 'false'
    pursuit = (CAND/'src/arena_autonomy/arena_autonomy/local_pursuit.py').read_text(encoding='utf-8')
    checks['node_parameter_default_false'] = pursuit.count("declare_parameter('g1_solo_straight_budget_trial',False)") == 1
    from arena_autonomy.local_follow import FollowConfig
    checks['follow_config_default_off'] = FollowConfig().solo_straight_budget_trial is False
    with (HERE.parent/'solo_jr60p20_baseline_v1/run1/follow_decisions.jsonl').open(encoding='utf-8') as stream:
        solo = json.loads(next(stream))['data']['follow_config']
    FollowConfig(**{**solo, 'solo_straight_budget_trial': True})
    checks['solo_run1_config_accepts_on'] = True
    with (HERE.parent/'moving_pass_withdrawal_v1/candidate_r13/run1/follow_decisions.jsonl').open(encoding='utf-8') as stream:
        traffic = json.loads(next(stream))['data']['follow_config']
    try:
        FollowConfig(**{**traffic, 'solo_straight_budget_trial': True})
        checks['traffic_r13_config_rejects_on'] = False
    except ValueError:
        checks['traffic_r13_config_rejects_on'] = True
    result = dict(scope=__doc__.strip(), cand=str(CAND), checks=checks, passed=all(checks.values()))
    (HERE/'wiring_check.json').write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n', encoding='utf-8')
    print(json.dumps(result['checks'], ensure_ascii=False), result['passed'])
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

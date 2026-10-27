"""시험기 CRLF를 보존한 선택형 플래그 연결과 현재 후속 출처 해시 갱신."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
path = ROOT/'scripts/validate_g1_traffic.py'
source = path.read_bytes()
updates = [
    ('pass_release_arguments, pass_release_evidence)', 'pass_release_arguments, pass_release_evidence, guard_curve_arguments)'),
    ("    parser.add_argument('--boundary-evidence',", "    parser.add_argument('--guard-curve-trial', action='store_true',\r\n                        help='GUARD 곡선 저속 후보. 기본 OFF, pass-release 필요')\r\n    parser.add_argument('--boundary-evidence',"),
    ('    release_arguments = pass_release_arguments(args.pass_release_trial,args.parallel_hold_trial)',
     '    release_arguments = pass_release_arguments(args.pass_release_trial,args.parallel_hold_trial)\r\n    guard_arguments = guard_curve_arguments(args.guard_curve_trial, args.pass_release_trial)'),
    ('*parallel_arguments, *release_arguments]', '*parallel_arguments, *release_arguments, *guard_arguments]'),
    ('evidence = pass_release_evidence(trace)', 'evidence = pass_release_evidence(trace, guard_curve=bool(guard_arguments))'),
    ("                'evidence':evidence,'original_traffic_passed':report['passed']}",
     "                'evidence':evidence,'original_traffic_passed':report['passed']}\r\n            if guard_arguments:\r\n                report['pass_release_trial']['guard_curve_arguments'] = guard_arguments"),
]
if b'--guard-curve-trial' not in source:
    for before, after in updates:
        before, after = before.encode('utf-8'), after.encode('utf-8')
        if source.count(before) != 1:
            raise ValueError(f'expected one replacement: {before[:80]!r}')
        source = source.replace(before, after)
    path.write_bytes(source)

def digest(relative):
    return hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()

contract = ROOT/'config/tests/partial_observation_r0_sha256.json'
data = json.loads(contract.read_text(encoding='utf-8'))
changes = ['src/arena_autonomy/arena_autonomy/local_follow.py',
           'src/arena_autonomy/arena_autonomy/local_pursuit.py',
           'src/arena_bringup/launch/demo.launch.py', 'src/arena_bringup/launch/local_pursuit.launch.py']
for name in changes:
    row = data['approved_changes'][name]
    row['after'] = digest(name)
    if 'GUARD 곡선' not in row['reason']:
        row['reason'] += '; 기본 OFF GUARD 곡선 후보 연결'
name = 'scripts/validate_g1_traffic.py'
data['approved_evaluation_scripts'][name]['after'] = digest(name)
if 'GUARD 곡선' not in data['approved_evaluation_scripts'][name]['reason']:
    data['approved_evaluation_scripts'][name]['reason'] += '; 명시 GUARD 곡선 시험 인자·진단 연결'
for name in ('scripts/g1_partial_trial.py', 'src/arena_autonomy/arena_autonomy/pass_release_trial.py'):
    data['dependencies'][name] = digest(name)
contract.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
child = ROOT/'config/tests/profile_bundle_selector_r0_sha256.json'
data = json.loads(child.read_text(encoding='utf-8'))
data['parent_contract_sha256'] = digest(contract.relative_to(ROOT))
child.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps({'connected': True, 'parent_sha256': data['parent_contract_sha256']}))

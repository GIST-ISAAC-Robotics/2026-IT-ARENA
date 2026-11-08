"""r40 종료 확인 후 미연결 코너 사본을 적용한다. 실행은 별도다."""
from pathlib import Path
import ast
import hashlib
import json
import subprocess
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
P = A.parent/'corner_path_support_probe_v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
parent = 'config/tests/adaptive_corridor_offset_r40_sha256.json'
contract = 'config/tests/current_corner_support_r41_sha256.json'
source = 'src/arena_autonomy/arena_autonomy/local_follow.py'
helper = 'src/arena_autonomy/arena_autonomy/corner_boundary_support.py'
assert not (ROOT/contract).exists() and not (ROOT/helper).exists() and not (A/'before.zip').exists()
state = subprocess.run(['pgrep', '-af', r'^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)'], capture_output=True, text=True)
assert state.returncode == 1, ('active simulation', state.stdout)
integrity = json.loads((A.parent/'entry_width_transition_v1/integrity.json').read_bytes())
assert integrity['changed'] == [] and integrity['snapshot_unchanged']
manifest = json.loads((P/'stage_manifest.json').read_bytes())
assert sha(ROOT/source) == manifest['before_sha256']
assert sha(P/'local_follow_candidate.py') == manifest['after_sha256']
assert sha(P/'candidate.py') == manifest['helper_sha256']
replay = json.loads((P/'stage_replay_result.json').read_bytes())
assert replay['calls'] == replay['baseline_exact_historical_calls'] == 1640
assert abs(replay['first_command_difference']['t']-34.62) < 1e-8
assert replay['counts'][1]['zero_command'] < replay['counts'][0]['zero_command']
for name in ('local_follow_candidate.py', 'candidate.py'): ast.parse((P/name).read_text())
chain = ROOT/'tests/test_source_provenance_r0_plus_b3.py'
text = chain.read_text(); needle = f"M3_SUCCESSORS += ('{parent}',)"
assert text.count(needle) == 1
with zipfile.ZipFile(A/'before.zip', 'x', zipfile.ZIP_DEFLATED) as z:
    for path in (source, chain.relative_to(ROOT).as_posix()): z.writestr(path, (ROOT/path).read_bytes())
c = dict(schema_version=1, scope='기본 OFF 교통 경계 갱신의 마지막 후보: 현재 두 면과 유효한 직전 코너 형상 대조. 미관측 공간/TTL/점유/보호는 변경하지 않음.',
    parent_contract=parent, parent_contract_sha256=sha(ROOT/parent),
    before_snapshot=(A/'before.zip').relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(A/'before.zip'),
    approved_changes={source: dict(before=sha(ROOT/source), after=sha(P/'local_follow_candidate.py'),
        reason='current two-face geometry supports retained corner when original boundary selection fails')},
    approved_evaluation_scripts={}, new_dependencies={helper: sha(P/'candidate.py')})
(ROOT/source).write_bytes((P/'local_follow_candidate.py').read_bytes())
(ROOT/helper).write_bytes((P/'candidate.py').read_bytes())
tests = (P/'test_candidate.py').read_text().replace('from candidate import propose', 'from arena_autonomy.corner_boundary_support import propose')
tests = tests.replace('A=Path(__file__).resolve().parent', "A=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-08/corner_path_support_probe_v1'")
(ROOT/'tests/test_corner_boundary_support.py').write_text(tests, encoding='utf8')
chain.write_text(text.replace(needle, needle+f"\nM3_SUCCESSORS += ('{contract}',)"), encoding='utf8')
(ROOT/contract).write_text(json.dumps(c, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
print('r41 applied; checks required before one live trial')

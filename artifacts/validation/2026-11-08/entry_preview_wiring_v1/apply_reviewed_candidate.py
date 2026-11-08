"""검토한 r36 사본만 적용. 먼저 전체 전제/구문/부모를 확인하고 시험 실행 중에는 거부한다."""
from pathlib import Path
import ast,hashlib,json,subprocess,sys,zipfile

A=Path(__file__).resolve().parent; ROOT=A.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((A/'wiring_manifest.json').read_bytes())
overlay=A.parent/'entry_preview_probe_v1/overlay/arena_autonomy'
before={**manifest['candidate_overlay']['before'],**manifest['before']}
after={**manifest['candidate_overlay']['after'],**manifest['after']}
parent='config/tests/piecewise_noisy_corner_r35_sha256.json'
contract_path='config/tests/adaptive_entry_preview_r36_sha256.json'
chain='tests/test_source_provenance_r0_plus_b3.py'
test_path='tests/test_adaptive_entry_preview.py'
assert not (ROOT/contract_path).exists() and not (ROOT/test_path).exists()
for name,h in before.items():assert sha(ROOT/name)==h,name
for name in set(after)-set(before):assert not (ROOT/name).exists(),name
staged={name:((A/'staged'/name) if name in manifest['after'] else overlay/Path(name).name).read_bytes()
        for name in after}
for name,data in staged.items():
    assert hashlib.sha256(data).hexdigest()==after[name],name
    ast.parse(data.decode(),filename=name)
text=(ROOT/chain).read_bytes().decode()
needle=f"M3_SUCCESSORS += ('{parent}',)"
assert text.count(needle)==1
chain_after=text.replace(needle,needle+f"\nM3_SUCCESSORS += ('{contract_path}',)").encode()
test=(A.parent/'entry_preview_probe_v1/test_adaptive_entry_preview_staged.py').read_text(encoding='utf8')
prefix='from pathlib import Path\nimport sys\nsys.path.insert(0, str(Path(__file__).parent/\'overlay\'))\n'
assert test.count(prefix)==1
test=test.replace(prefix,'').replace('별도 overlay의 표적 회귀. 실행 중인 A/B의 src는 변경하지 않는다.','짧은 목표점 후보의 현재 제어 모듈 표적 회귀.')
ast.parse(test)
parent_data=json.loads((ROOT/parent).read_bytes())
for name,row in parent_data['approved_changes'].items():assert sha(ROOT/name)==row['after']
if sys.platform.startswith('linux'):
    probe=subprocess.run(['pgrep','-af',r'^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)'],capture_output=True,text=True)
else:
    probe=subprocess.run(['wsl','-e','pgrep','-af',r'^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)'],capture_output=True,text=True)
assert probe.returncode==1,('simulation active or process check failed',probe.stdout,probe.stderr)
if '--check' in sys.argv:
    print('ready to apply',len(staged),'modules; no simulation');raise SystemExit(0)
assert not (A/'before.zip').exists()
with zipfile.ZipFile(A/'before.zip','x',zipfile.ZIP_DEFLATED) as z:
    for name in (*before,chain):z.writestr(name,(ROOT/name).read_bytes())
contract=dict(schema_version=1,scope='기본OFF adaptive_entry_preview: 기존 저속 우회 경로의 짧은 목표점과 정지 조향 준비. 점유/제동/관측/STOP/독립 보호 불변.',
    parent_contract=parent,parent_contract_sha256=sha(ROOT/parent),
    before_snapshot=(A/'before.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(A/'before.zip'),
    approved_changes={name:dict(before=before[name],after=after[name],reason='저속 기억 우회 경로에만 선택형 짧은 목표점 및 정지 정렬, 기본OFF 연결') for name in before},
    approved_evaluation_scripts={},new_dependencies={name:after[name] for name in set(after)-set(before)})
for name,data in staged.items():(ROOT/name).write_bytes(data)
(ROOT/test_path).write_bytes(test.encode());(ROOT/chain).write_bytes(chain_after)
(ROOT/contract_path).write_bytes((json.dumps(contract,ensure_ascii=False,indent=2)+'\n').encode())
print('applied r36',len(staged),'modules; tests/build/live run not performed by this script')

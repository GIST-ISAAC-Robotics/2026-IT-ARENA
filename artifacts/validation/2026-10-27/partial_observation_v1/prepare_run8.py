"""run7 실패 사본을 보존하고 보존 목록 수정만 이어 등록한다."""
from pathlib import Path
import hashlib
import json
import subprocess
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sha = lambda b: hashlib.sha256(b).hexdigest()
path = 'scripts/validate_pipeline_faults_ros.py'
original = subprocess.run(['git', 'show', '0e6c28a:'+path], cwd=ROOT,
                          capture_output=True, check=True).stdout
with zipfile.ZipFile(HERE/'before_main.zip') as old, zipfile.ZipFile(
        HERE/'before_main_v2.zip', 'x', zipfile.ZIP_DEFLATED) as new:
    for name in old.namelist():
        new.writestr(name, old.read(name))
    new.writestr(path, original)
contract_path = ROOT/'config/tests/partial_observation_r0_sha256.json'
contract = json.loads(contract_path.read_text(encoding='utf-8'))
contract['before_snapshot'] = (HERE/'before_main_v2.zip').relative_to(ROOT).as_posix()
contract['before_snapshot_sha256'] = sha((HERE/'before_main_v2.zip').read_bytes())
contract['approved_evaluation_scripts'][path] = dict(before=sha(original),
    after=sha((ROOT/path).read_bytes()), reason='새 보호 모듈의 실제 ROS 고장 시험 보존 목록 보충')
contract_path.write_text(json.dumps(contract, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
snapshot = (HERE/'snapshot.py').read_text(encoding='utf-8').replace(
    "'source_manifest.json'", "'run8_source_manifest.json'").replace(
    "'source_snapshot.zip'", "'run8_source_snapshot.zip'")
(HERE/'run8_snapshot.py').write_text(snapshot, encoding='utf-8')
script = (HERE/'run7_trial.sh').read_text(encoding='utf-8').replace('run7_', 'run8_').replace(
    '"$base/snapshot.py"', '"$base/run8_snapshot.py"').replace(
    '-m pytest -q tests >',
    '-m pytest -q tests/test_official_start_ros_fixture.py tests/test_pipeline_fault_validation.py tests/test_source_provenance_r0_plus_b3.py >')
(HERE/'run8_trial.sh').write_text(script, encoding='utf-8', newline='\n')
print('run8 ready: manifest-only repair and targeted regression; no driving-policy changes')

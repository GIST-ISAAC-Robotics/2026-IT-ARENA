"""후반부 결측 후보의 현재 출처/의존성만 갱신. 원래 before·스냅샷은 보존한다."""
import hashlib
import json
from pathlib import Path
import runpy

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
runpy.run_path(str(HERE/'wire_trial.py'))


def digest(name):
    return hashlib.sha256((ROOT/name).read_bytes()).hexdigest()


parent = 'config/tests/partial_observation_r0_sha256.json'
data = json.loads((ROOT/parent).read_text(encoding='utf-8'))
name = 'src/arena_autonomy/arena_autonomy/rear_fringe_trial.py'
data['dependencies'][name] = digest(name)
name = 'scripts/validate_pipeline_faults_ros.py'
data['approved_evaluation_scripts'][name]['after'] = digest(name)
if '후반부 결측' not in data['approved_evaluation_scripts'][name]['reason']:
    data['approved_evaluation_scripts'][name]['reason'] += '; 후반부 결측 분류 의존성 보존'
(ROOT/parent).write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
child = ROOT/'config/tests/profile_bundle_selector_r0_sha256.json'
data = json.loads(child.read_text(encoding='utf-8'))
data['parent_contract_sha256'] = digest(parent)
for name in ('scripts/profile_follow_trace.py', 'scripts/prepare_jetson_bundle.py'):
    data['approved_evaluation_scripts'][name]['after'] = digest(name)
    if '후반부 결측' not in data['approved_evaluation_scripts'][name]['reason']:
        data['approved_evaluation_scripts'][name]['reason'] += '; 후반부 결측 후보 의존성 추가'
child.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print('Current dependency provenance updated; historical snapshots unchanged.')

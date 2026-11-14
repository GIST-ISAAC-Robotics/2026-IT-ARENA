"""저장 r555 미출발 report로 수집기의 unavailable 분기만 확인한다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

A = Path(__file__).resolve().parent.parent
ROOT = A.parents[3]
OLD = A.parent / 'r555_sparse_actor_ready_v1'
FIXTURE = A / 'preparation_checks/startup_fixture'
sys.path[:0] = [str(A), str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
before = sha(OLD/'run1/report.json')
for name in ['collect_result', 'assess']:
    spec = importlib.util.spec_from_file_location('r560_'+name, A/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.A = FIXTURE
    module.ROOT = ROOT
    module.main()
summary = json.loads((FIXTURE/'execution_summary.json').read_text())
assessment = json.loads((FIXTURE/'assessment_summary.json').read_text())
assert summary['summary_available'] is False
assert assessment['assessment_available'] is False and assessment['existing_supplement_passed'] is None
old = json.loads((A/'recipe_provenance.json').read_text())['old_r555_files_sha256']
changed = [p for p,h in old.items() if sha(OLD/p.replace('\\','/')) != h]
assert not changed
result = dict(passed=True, scope='Saved startup-error fixture only; no controller replay or drive',
              summary_unavailable_not_pass=True, assessment_unavailable_not_pass=True,
              selected_measured_vehicle=assessment['selected_vehicle_config'],
              old_r555_files_checked=len(old), old_r555_changed=changed,
              raw_report_unchanged=sha(OLD/'run1/report.json')==before)
(A/'preparation_checks/result.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))

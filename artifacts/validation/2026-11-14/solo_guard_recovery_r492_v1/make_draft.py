"""r485 recipe의 경로만 새 초안으로 복사한다. 동결/build/ROS 실행 없음."""
import ast
import hashlib
import json
import shutil
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = A.parent / 'solo_variation_recovery_r485_v1'

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

assert not (A / 'draft_provenance.json').exists()
names = ['solo_runner.py', 'solo_evaluation.py', 'solo_finalize.py', 'facility_geometry.py',
    'facilities_only.sdf', 'expected_header.json', 'audit_runtime.py', 'header_comparison.py',
    'course.launch.py', 'launch/demo.launch.py', 'launch/local_pursuit.launch.py',
    'launch/simulation.launch.py', 'prepare.py', 'variation_launch.py', 'launch_adapter.py',
    'run.py', 'stop_owned.py', 'prepare_execution.py', 'runtime_preflight.py',
    'execution/execute_once.sh', 'execution/stop_once.sh', 'execution/observe.py',
    'execution/check_owned_processes.py', 'execution/prepare_once.sh',
    'connection_v1/start_left20_yaw_plus3.sdf', 'prepared/manifest.json']
exact = {'expected_header.json', 'facilities_only.sdf',
    'connection_v1/start_left20_yaw_plus3.sdf', 'prepared/manifest.json'}
records = []
for name in names:
    src, dst = OLD / name, A / name
    dst.parent.mkdir(parents=True, exist_ok=True)
    if name in exact:
        shutil.copyfile(src, dst)
    else:
        text = src.read_text(encoding='utf-8').replace(OLD.name, A.name)
        text = text.replace('r485', 'r492').replace('R485', 'R492')
        dst.write_text(text, encoding='utf-8', newline='\n')
    records.append(dict(path=name, source=str(src.relative_to(ROOT)),
        before_sha256=sha(src), draft_sha256=sha(dst), byte_exact=name in exact))
config = json.loads((OLD / 'config.json').read_bytes())
config['scope'] = 'r492 pending: same r485 +20mm/+3deg actor0; one changed-guard trial after root integration/review; no execution authorized'
config['launch']['entry_g1'] = (A / 'course.launch.py').relative_to(ROOT).as_posix()
(A / 'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
(A / 'source_selection.json').write_text(json.dumps(dict(common_integration_confirmed=False,
    freeze_authorized=False, wsl_owner_confirmed=False, policy_changes=[], noncontrol_changes=[],
    expected_native_sha256=None, status='pending_root_r489_integration_and_source_delta_review'),
    ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
for p in A.rglob('*.py'):
    ast.parse(p.read_bytes())
(A / 'draft_provenance.json').write_text(json.dumps(dict(
    source_case='r485 start_left20_yaw_plus3', copied_files=records,
    source_prepared_sha256=sha(OLD / 'execution_prepared.json'),
    source_archive_sha256=sha(OLD / 'execution_sources.zip'),
    source_report_sha256=sha(OLD / 'cases/start_left20_yaw_plus3/run1/report.json'),
    control_sources_copied=False, source_frozen=False, build_executed=False,
    wsl_executed=False, simulation_started=False, prepare_templates_need_source_selection_update=True),
    ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(dict(copied=len(records), ast_checked=True, frozen=False, execution_authorized=False)))

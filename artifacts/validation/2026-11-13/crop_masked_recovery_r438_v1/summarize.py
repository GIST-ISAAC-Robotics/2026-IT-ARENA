"""회수한 계산 결과를 집계한다. 입력/정책을 다시 실행하지 않는다."""
from pathlib import Path
import hashlib
import json
import math
import statistics
import zipfile

root=Path(__file__).resolve().parent;repo=root.parents[3]
report=json.loads((root/'result_v1/report.json').read_bytes())
rows=[json.loads(line) for line in (root/'result_v1/rows.jsonl').open(encoding='utf-8')]
manifest=json.loads((root/'preparation.json').read_bytes())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
checks=dict(r437_archive=sha(root.parent/'applied_limit_drive_r437_v1/sources.zip')==manifest['source_archive_sha256'],
    r437_trace=sha(root.parent/'applied_limit_drive_r437_v1/run1/follow_decisions.jsonl')==manifest['trace_sha256'],
    r426_inputs=sha(root.parent/'crop_current_reach_r426_v1/inputs/currents.jsonl')==manifest['r426_currents_sha256'],
    r426_expected=sha(root.parent/'crop_current_reach_r426_v1/verify_v1/conditional-full.zip')==manifest['r426_expected_archive_sha256'],
    stage_lf_text=hashlib.sha256((root/'stage/fixed_target_candidate.py').read_bytes().replace(b'\r\n',b'\n')).hexdigest()==manifest['stage_sha256'],
    local_inputs=all(sha(root/'inputs'/name)==want for name,want in manifest['inputs'].items()))
with zipfile.ZipFile(root/'frozen_packages.zip') as z:
    checks['frozen_archive']=all(hashlib.sha256(z.read(name)).hexdigest()==want for name,want in manifest['frozen_package_files'].items())
shared=[name for name,want in manifest['frozen_package_files'].items() if sha(repo/name)!=want]
checks['shared_core_exact_except_r434_ui']=set(shared)=={
    'src/arena_autonomy/arena_autonomy/race_ops_node.py','src/arena_autonomy/arena_autonomy/race_ops_page.py'}
cost=sorted(row['baseline_full_ms']+row['trials']['0.1']['additional_current_check_ms'] for row in rows if row['dataset']=='r437' and row['eligible'])
summary=dict(saved_run_passed=report['passed'],checks=checks,shared_differences_from_r437=shared,
    stage_source_audit=dict(raw_sha256=sha(root/'stage/fixed_target_candidate.py'),prepared_lf_text_sha256=manifest['stage_sha256'],
        explanation='prepare hashed LF text before Windows write_text emitted CRLF; LF-normalized content exactly matches; stage unchanged'),
    combined_eligible72_cost_ms=dict(minimum=min(cost),median=statistics.median(cost),p95=cost[math.ceil(.95*len(cost))-1],maximum=max(cost)),
    early_cap_would_intercept_existing_positive=sum(row['dataset']=='r426' and 0<=row['measured_speed_mps']<=.15
        and row['baseline']['crop'].get('applied') is True and row['baseline']['speed_mps']>.1 for row in rows),
    accepted_at_015=[row['trace_seq'] for row in rows if row['dataset']=='r437' and row['trials']['0.15']['result']['speed_mps']>0],
    observed_obstacle_gap_hold_cases=sum(row['baseline']['reason']=='rebase_current_gap_hold' for row in rows),
    foreground_exitcode=0,post_exit_process_query='WSL Service E_UNEXPECTED twice; no restart attempted',
    representative=next(row for row in rows if row['dataset']=='r437' and row['trace_seq']==643))
assert all(checks.values())
(root/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k!='representative'},ensure_ascii=False))

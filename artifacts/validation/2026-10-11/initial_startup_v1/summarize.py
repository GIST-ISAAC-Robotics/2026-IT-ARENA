"""등록 3회 전체(시작 전 실패 포함)를 별도 상태로 모은다."""
import json
import hashlib
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_initial_startup import analyze_run, compare, bounded_confirmation, world_and_model
from analyze_evaluator_gc import source_snapshot_valid
from analyze_apply_trace_v2 import registered_sources

r5 = json.loads((ROOT/'config/tests/scheduled_command_r5_sha256.json').read_text())
r6 = json.loads((ROOT/'config/tests/scheduled_command_r6_sha256.json').read_text())
a = analyze_run(BASE/'s1_service',r5)
b = analyze_run(BASE/'e2_embedded',r6)
failure = json.loads((BASE/'e1_embedded/report.json').read_text())
status = json.loads((BASE/'status_e1_embedded.json').read_text())['cases'][0]
failed_path = BASE/'e1_embedded'
raw_ok = {}
for key,value in failure.items():
    if isinstance(value,dict) and 'sha256' in value and 'samples' in value:
        p = failed_path/(key+'.jsonl')
        raw_ok[key] = p.is_file() and hashlib.sha256(p.read_bytes()).hexdigest() == value['sha256']
last = failure['evaluation_startup']['probes'][-1]
failure_checks = {'declared_failure':failure['passed'] is False and failure['input_valid'] is False and status['rc'] == 1,
                 'expected_readiness_error':failure.get('error') == 'TimeoutError: paused startup readiness timeout',
                 'no_release':failure['evaluation_startup']['released'] is False,
                 'no_drive_or_first_update':failure['requested_commands']['samples'] == 0 and failure['plugin_command_events']['samples'] == 0,
                 'four_contacts_observed':last['/eval/wheels/contact']['publishers'] == 4,
                 'shutdown':failure['shutdown_clean'] is True and status['residual_after'] == [],
                 'gc_restored':failure['evaluation_gc']['restored'] is True,
                 'sources_registered':registered_sources(failure['source_sha256'],r5),
                 'source_zip':source_snapshot_valid(failed_path/'source_snapshot.zip',failure['source_sha256']),
                 'raw_integrity':all(raw_ok.values()),
                 'embedded_shape_preserved':world_and_model(failed_path,failure)['passed']}
result = {'runs':[a,b], 'failed_attempt':{'name':'e1_embedded','checks':failure_checks,
            'evidence_verified':all(failure_checks.values()), 'error':failure['error']},
          'comparison':compare([a,b]), 'bounded_confirmation':bounded_confirmation(a,b,r6)}
with (BASE/'analysis_all.json').open('x',encoding='utf-8') as f: json.dump(result,f,ensure_ascii=False,indent=2)
print(json.dumps({'failed_evidence_verified':all(failure_checks.values()), **result['bounded_confirmation']},indent=2))

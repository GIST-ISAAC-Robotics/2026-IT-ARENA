"""완료 결과의 출처/해시 검토. integrity 통과와 주행 성공을 별도 보고한다."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile
from datetime import datetime, timezone

root = Path(__file__).resolve().parents[4]
base = root/'artifacts/validation/2026-10-05'
final = '--final' in sys.argv
results = []
cases = [
    ('g1_synthetic_v2', 'source_sha256', False),
    ('g1_gazebo_solo_v1', 'input_sha256', False),
    ('g1_synthetic_v3', 'source_sha256', not final),
    ('g1_gazebo_solo_v2', 'input_sha256', not final),
]
if final:
    cases += [('g1_synthetic_v4', 'source_sha256', True), ('g1_gazebo_solo_v3', 'input_sha256', True)]
for name, hash_key, current in cases:
    folder = base/name
    data = json.loads((folder/'report.json').read_text(encoding='utf-8'))
    hashes = data[hash_key]
    mismatch, current_mismatch, trace_mismatch = [], [], []
    with zipfile.ZipFile(folder/'source_snapshot.zip') as archive:
        for rel, expected in hashes.items():
            if hashlib.sha256(archive.read(rel)).hexdigest() != expected:
                mismatch.append(rel)
            if current and hashlib.sha256((root/rel).read_bytes()).hexdigest() != expected:
                current_mismatch.append(rel)
    for run in data.get('results', []):
        if hashlib.sha256((folder/run['trace']).read_bytes()).hexdigest() != run['trace_sha256']:
            trace_mismatch.append(run['trace'])
    result = {'report': str((folder/'report.json').relative_to(root)),
        'report_sha256': hashlib.sha256((folder/'report.json').read_bytes()).hexdigest(),
        'archive_sha256': hashlib.sha256((folder/'source_snapshot.zip').read_bytes()).hexdigest(),
        'sources': len(hashes), 'archive_mismatches': mismatch, 'current_source_comparison_required': current,
        'current_mismatches': current_mismatch, 'traces': len(data.get('results', [])), 'trace_mismatches': trace_mismatch,
        'operational_passed': data['passed']}
    if 'results' in data:
        result['g1_passes'] = sum(row['passed'] for row in data['results'] if row['mode'] == 'g1')
        result['g1_cases'] = sum(row['mode'] == 'g1' for row in data['results'])
    else:
        result.update({key: data.get(key) for key in ('progress_m', 'error', 'stop_stable',
            'collision_samples', 'stop_collision_samples', 'shutdown_clean', 'remaining_test_pids',
            'g1_runtime_verified', 'actuation_passed', 'actuation_diagnostics')})
    results.append(result)

allowed = {'scripts/validate_basic_autonomy.py', 'src/arena_autonomy/arena_autonomy/local_pursuit.py'}
diff = subprocess.check_output(['git', 'diff', '--name-only', '25dea6045098105a1289ac8ffdb9b2b366c16edb',
    '--', 'src', 'scripts', 'tests', 'config'], cwd=root, text=True).splitlines()
unexpected = sorted(set(diff)-allowed)
integrity = not unexpected and all(not row['archive_mismatches'] and not row['current_mismatches']
    and not row['trace_mismatches'] for row in results)
out = {'reviewed_at_utc': datetime.now(timezone.utc).isoformat(),
    'integrity_passed': integrity, 'scope': 'saved source/archive/trace integrity and preserved tracked baseline only',
    'does_not_override_run_verdicts': True, 'tracked_baseline_changes': diff, 'unexpected_baseline_changes': unexpected,
    'results': results}
path = Path(__file__).with_name('collection_final.json' if final else 'collection.json')
if path.exists():
    raise FileExistsError(path)
path.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({'integrity_passed': integrity, 'runs': len(results), 'output': str(path)}, ensure_ascii=False))
raise SystemExit(0 if integrity else 1)

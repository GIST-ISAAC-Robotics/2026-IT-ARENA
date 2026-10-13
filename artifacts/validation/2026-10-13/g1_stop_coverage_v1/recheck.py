"""보존한 5개 주행을 새 STOP 자료 완비 감사로만 다시 읽는다. 재주행/원본 변경 없음."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path.cwd()
BASE = Path(__file__).resolve().parent
CASES = (
    ('left_s0', 'gapped_boundary_v3/side_left_s0', True),
    ('initial_s0', 'g1_candidate_regression_v1/initial_s0', True),
    ('right_s0', 'g1_right_candidate_v1/side_right_s0', True),
    ('curve_s0', 'g1_curve_candidate_v1/first_curve_s0', True),
    ('left_s1_failed_hold', 'g1_position_matrix_v1/side_left_s1', False),
)
results = []
for label, name, expected in CASES:
    run = ROOT/'artifacts/validation/2026-10-13'/name
    output = BASE/(label+'.json')
    names = ('report.json', 'trace.json', 'autonomy_status.jsonl', 'ego_poses.jsonl', 'stop_trace.json')
    hashes = {n: hashlib.sha256((run/n).read_bytes()).hexdigest() for n in names}
    command = [sys.executable, '-B', str(ROOT/'scripts/audit_g1_traffic_coverage.py'),
               str(run), '--output', str(output)]
    child = subprocess.run(command, capture_output=True, text=True)
    with (BASE/(label+'.log')).open('x', encoding='utf-8') as stream:
        stream.write(child.stdout+child.stderr)
    report = json.loads(output.read_text()) if output.exists() else {}
    unchanged = all(hashlib.sha256((run/n).read_bytes()).hexdigest() == digest for n, digest in hashes.items())
    results.append(dict(case=label, original_run_passed=json.loads((run/'report.json').read_text()).get('passed'),
                        expected_audit_passed=expected, audit_passed=report.get('passed'),
                        audit_exit=child.returncode, checks=report.get('checks'),
                        stop_coverage=report.get('stop_pose_coverage'), inputs_unchanged=unchanged,
                        passed=unchanged and report.get('passed') is expected and child.returncode==(0 if expected else 1)))
summary = dict(scope='saved_input_reaudit_only_not_new_driving', cases=results,
               passed=all(r['passed'] for r in results))
with (BASE/'recheck_report.json').open('x', encoding='utf-8') as stream:
    json.dump(summary, stream, ensure_ascii=False, indent=2)
    stream.write('\n')
print(json.dumps(summary, ensure_ascii=False))
raise SystemExit(0 if summary['passed'] else 2)

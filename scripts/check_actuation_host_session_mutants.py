#!/usr/bin/env python3
"""호스트 세션 후보의 핵심 반례 검출력. 별도 사본만 변이하고 원본은 보존한다."""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'scripts/actuation_host_session.py'
TEST = ROOT/'tests/test_actuation_host_session.py'
MUTANTS = [
    ('arm_revocation_removed',
     '    def _revoke(self):\n        self._arm_generation = self._arm_barrier = None',
     '    def _revoke(self):\n        pass'),
    ('stop_does_not_preempt', "            self._finish('superseded_by_stop_unconfirmed')", '            pass'),
    ('stop_intent_not_latched', '            self._stop_requested = True', '            self._stop_requested = False'),
    ('ack_request_id_ignored', "p is None or payload['request'] != p.request", 'p is None'),
    ('ack_timeout_ignored', 'self.pending is not None and now.upper_ns >= self.pending.deadline_ns',
     'False and self.pending is not None and now.upper_ns >= self.pending.deadline_ns'),
    ('negative_ack_accepted', "        if not payload['accepted']:\n", "        if False:\n"),
    ('ack_reason_ignored', "        if payload['reason'] != CONTROL_REASONS[p.kind]:\n", '        if False:\n'),
    ('ack_order_ignored', '        if received.lower_ns < p.started.upper_ns:', '        if False:'),
    ('foreign_connection_accepted', '        if connection_id != self._observer.connection_id:', '        if False:'),
    ('arm_before_both_receipts',
     '            self._arm_barrier = self._last_event',
     '            self._arm_barrier = HostTimeWindow(self._last_event.host_epoch, self._last_event.lower_ns, self._last_event.lower_ns)'),
    ('arm_barrier_ignores_processing_delay', '            self._arm_barrier = self._last_event',
     '            self._arm_barrier = self._confirmation.received'),
    ('source_watermark_removed',
     'self._source_watermark is not None and source.lower_ns <= self._source_watermark.upper_ns',
     'False and self._source_watermark is not None and source.lower_ns <= self._source_watermark.upper_ns'),
    ('clear_does_not_release_local_stop', '            self._stop_requested = False\n        self._finish(',
     '            pass\n        self._finish('),
]

# import 경로 경쟁을 피하기 위해 시험 대상 사본을 정확한 module 이름으로 먼저 적재한다.
BOOTSTRAP = r'''
import importlib.util, json, pathlib, sys
candidate, test, failures = sys.argv[1:]
spec = importlib.util.spec_from_file_location('actuation_host_session', candidate)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
import pytest
class Failures:
    rows = []
    def pytest_runtest_makereport(self, item, call):
        if call.excinfo is not None:
            self.rows.append(dict(test=item.nodeid, stage=call.when, error_type=call.excinfo.typename))
plugin = Failures()
code = pytest.main(['-q', '-x', '-p', 'no:cacheprovider', '-k', 'not actual_cpp and not production_nodes', test], plugins=[plugin])
pathlib.Path(failures).write_text(json.dumps(plugin.rows, indent=2), encoding='utf-8')
raise SystemExit(code)
'''


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    sources = (SOURCE, TEST, Path(__file__).resolve())
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in sources}
    original = SOURCE.read_text(encoding='utf-8')
    rows = []
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    for name, old, new in [('unmutated_copy', '', '')]+MUTANTS:
        folder = output/name
        folder.mkdir()
        if name != 'unmutated_copy' and original.count(old) != 1:
            rows.append(dict(name=name, detected=False, error='pattern_not_unique'))
            continue
        changed = original if name == 'unmutated_copy' else original.replace(old, new)
        ast.parse(changed)  # 문법 오류를 검출 성공으로 세지 않는다.
        copy = folder/'actuation_host_session.py'
        copy.write_text(changed, encoding='utf-8')
        run = subprocess.run([sys.executable, '-B', '-c', BOOTSTRAP, str(copy), str(TEST), str(folder/'failures.json')],
            cwd=ROOT, env=env, text=True, capture_output=True, timeout=120)
        (folder/'pytest.log').write_text(run.stdout+run.stderr, encoding='utf-8')
        failures = json.loads((folder/'failures.json').read_text()) if (folder/'failures.json').exists() else []
        assertion_failed = run.returncode == 1 and bool(failures) and all(
            f['stage'] == 'call' and f['error_type'] == 'AssertionError' for f in failures)
        rows.append(dict(name=name, pytest_exit=run.returncode, failures=failures,
                         detected=assertion_failed if name != 'unmutated_copy' else None))
    control = rows[0].get('pytest_exit') == 0
    unchanged = all(digest(ROOT/p) == h for p, h in hashes.items())
    detected = sum(row.get('detected', False) is True for row in rows[1:])
    report = dict(control_passed=control, detected=detected, mutants=len(MUTANTS),
        source_sha256=hashes, source_unchanged=unchanged, results=rows,
        passed=control and unchanged and detected == len(MUTANTS), ready_to_drive=False)
    (output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in ('results', 'source_sha256')}))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())

"""수정 전 묶음의 입력 감사 허점을 재현한다. 소스/입력/결과를 덮어쓰지 않는다."""
import argparse
from copy import deepcopy
import importlib.util
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--auditor', type=Path, required=True)
parser.add_argument('--example', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
spec = importlib.util.spec_from_file_location('previous_capture_audit', args.auditor)
old = importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)
data = [json.loads(line) for line in args.example.read_text().splitlines()]
cases = []
for name in ('sequence_hole', 'truncated_window', 'missing_encoder_after_reboot'):
    header, rows = deepcopy(data[0]), deepcopy(data[1:])
    if name == 'sequence_hole':
        for row in rows[4:]:
            if row['kind'] == 'imu':
                row['seq'] += 3
    elif name == 'truncated_window':
        header['externally_stationary_windows'][0]['first_us'] = 0
        header['externally_stationary_windows'][0]['last_us'] = 3_000_000
    else:
        rows.append({**rows[-1], 'boot': 'new', 'seq': 0,
                     'capture_us': 0, 'receive_ns': rows[-1]['receive_ns']+1_000_000})
    result = old.audit(header, rows)
    cases.append(dict(name=name, record_integrity_valid=result['record_integrity_valid'],
                      candidate_only=result['stationary_windows'][0]['candidate_only'],
                      issues=result['issues']))
result = dict(source=str(args.auditor), source_sha256=old.digest(args.auditor),
              cases=cases, scope='known_defect_reproduction_not_validation_success')
with args.output.open('x', encoding='utf-8') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result))

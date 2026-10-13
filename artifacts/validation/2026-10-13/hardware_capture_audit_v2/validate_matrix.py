"""고정 합성 입력의 품질 분리 확인. 실제 교정·드라이버 시험이 아니다."""
from copy import deepcopy
import json
from pathlib import Path
import zipfile

import audit_hardware_capture as module

root = Path.cwd()
base = root/'artifacts/validation/2026-10-13/hardware_capture_audit_v2'
paths = [root/'scripts/audit_hardware_capture.py', root/'tests/test_hardware_capture_audit.py',
         root/'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py',
         root/'config/tests/hardware_capture_nominal_v1.jsonl', Path(__file__).resolve()]
hashes = {str(p.relative_to(root)): module.digest(p) for p in paths}
with zipfile.ZipFile(base/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
    for path in paths:
        bundle.write(path, path.relative_to(root))
data = [json.loads(line) for line in paths[3].read_text().splitlines()]
cases = []
for name in ('explicit_quality', 'requirements_missing', 'sequence_hole', 'truncated_window',
             'missing_encoder_after_reboot', 'long_capture_gap', 'changing_counts'):
    header, rows = deepcopy(data[0]), deepcopy(data[1:])
    window = header['externally_stationary_windows'][0]
    window['review_requirements'] = dict(minimum_span_us=40000, max_sample_gap_us=10000,
                                         minimum_imu_samples=5, minimum_encoder_samples=5)
    if name == 'requirements_missing':
        del window['review_requirements']
    elif name == 'sequence_hole':
        for row in rows[4:]:
            if row['kind'] == 'imu':
                row['seq'] += 3
    elif name == 'truncated_window':
        window.update(first_us=0, last_us=3_000_000)
    elif name == 'missing_encoder_after_reboot':
        rows.append({**rows[-1], 'boot': 'new', 'seq': 0,
                     'capture_us': 0, 'receive_ns': rows[-1]['receive_ns']+1_000_000})
    elif name == 'long_capture_gap':
        window['last_us'] = 160000
        for row in rows[4:]:
            row['capture_us'] += 20000
            row['receive_ns'] += 20000000
    elif name == 'changing_counts':
        rows[4]['motor_counts'] += 1
    report = module.audit(header, rows)
    candidate = report['stationary_windows'][0]
    expected_candidate = name == 'explicit_quality'
    expected_integrity = name != 'missing_encoder_after_reboot'
    matched = (candidate['candidate_only'] == expected_candidate
               and report['record_integrity_valid'] == expected_integrity
               and not report['ready_to_drive'] and not candidate['calibration_ready']
               and not candidate['applied'])
    cases.append(dict(name=name, matched=matched, report=report))
unchanged = all(module.digest(root/name) == value for name, value in hashes.items())
passed = unchanged and all(case['matched'] for case in cases)
result = dict(passed=passed, cases=cases, source_sha256=hashes, sources_unchanged=unchanged,
              physical_calibration_verified=False, scope='synthetic_file_quality_audit_only')
with (base/'matrix.json').open('x', encoding='utf-8') as stream:
    json.dump(result, stream, indent=2, allow_nan=False)
print(json.dumps(dict(passed=passed, cases=len(cases), sources_unchanged=unchanged)))
raise SystemExit(0 if passed else 1)

"""v1 시험 래퍼의 반환값 누락만 고친 별도 보존 실행."""
import hashlib
import json
import os
from pathlib import Path
import shutil
from validate_sensor_probe import trial

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
assert not (HERE/'report.json').exists()
sources = ['scripts/record_sensor_bag.py', 'scripts/validate_sensor_probe.py',
           'scripts/audit_sensor_probe.py', 'scripts/audit_sensor_bag.py',
           'scripts/sensor_probe_delivery.py', 'scripts/sensor_recording_profile.py',
           'scripts/rosbag_writer_compat.py', 'scripts/recording_guard.py',
           'src/arena_vehicle_interface/arena_vehicle_interface/bag_contract.py',
           'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py']
before = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
for p in sources:
    dest = HERE/'sources'/p
    dest.parent.mkdir(parents=True, exist_ok=True)
    with dest.open('xb') as out, (ROOT/p).open('rb') as original:
        shutil.copyfileobj(original, out)
rows = []
for fault in ('write', 'close'):
    os.environ['ARENA_SENSOR_PROBE_WRITER_FAULT'] = fault
    result = trial(HERE/fault, 'writer_'+fault, recorder_entrypoint=HERE/'recorder.py',
                   expected_recorder_error='injected_writer_'+fault+'_failure')
    rows.append(dict(case=fault, passed=result['passed'], checks=result['checks']))
after = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
report = dict(passed=all(r['passed'] for r in rows) and before == after,
              cases=rows, source_unchanged=before == after, source_sha256=before,
              physical_disk_error_reproduced=False, injection='wrapper OSError',
              hardware_verified=False, ready_to_drive=False)
with (HERE/'report.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print(json.dumps(report))
raise SystemExit(0 if report['passed'] else 1)

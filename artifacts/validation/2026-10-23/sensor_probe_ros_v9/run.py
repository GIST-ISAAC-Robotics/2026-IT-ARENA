"""5개 입력, 원래 0.05 s/0.25 s 조건에 명시 SHM 16 MiB만 선택한다."""
import hashlib
import json
from pathlib import Path
import shutil
import sys
from transport_support import configure, inspect
from validate_sensor_probe import trial
from analyze_sensor_probe_timing import analyze

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
assert not (HERE/'report.json').exists()
sources = ['scripts/record_sensor_bag.py', 'scripts/validate_sensor_probe.py',
           'scripts/audit_sensor_probe.py', 'scripts/audit_sensor_bag.py',
           'scripts/analyze_sensor_probe_timing.py', 'scripts/sensor_probe_delivery.py',
           'scripts/sensor_recording_profile.py', 'scripts/rosbag_writer_compat.py',
           'scripts/recording_guard.py', 'scripts/replay_transport.py',
           'src/arena_vehicle_interface/arena_vehicle_interface/bag_contract.py',
           'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py']
before = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
for p in sources:
    dest = HERE/'sources'/p
    dest.parent.mkdir(parents=True, exist_ok=True)
    with dest.open('xb') as stream, (ROOT/p).open('rb') as original:
        shutil.copyfileobj(original, stream)
result = trial(HERE/'full_input', 'full_input_shm_16m', full_input=True,
               sample_wait_seconds=.05, drain_seconds=.25,
               recorder_entrypoint=HERE/'recorder.py', before_ros_init=configure,
               node_observer=inspect)
after = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
report = dict(passed=result['passed'] and before == after, source_sha256=before,
              source_unchanged=before == after, checks=result['checks'],
              actual_transport_audited_both=True, global_settings_changed=False,
              default_changed=False, hardware_verified=False, jetson_verified=False,
              ready_to_drive=False)
with (HERE/'report.json').open('x') as stream:
    json.dump(report, stream, indent=2)
with (HERE/'timing.json').open('x') as stream:
    json.dump(analyze(HERE/'full_input/sensor_recording'), stream, indent=2)
print(json.dumps(report))
raise SystemExit(0 if report['passed'] else 1)

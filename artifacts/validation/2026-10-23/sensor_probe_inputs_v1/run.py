"""저장 5입력을 파일로만 검사한다. 소스/입력은 덮어쓰지 않는다."""
import hashlib
import json
from pathlib import Path
import shutil

import rclpy
from inspect_sensor_probe_inputs import analyze

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
INPUT = ROOT/'artifacts/validation/2026-10-23/sensor_probe_ros_v9/full_input/sensor_recording'
sources = ['scripts/inspect_sensor_probe_inputs.py','scripts/audit_sensor_probe.py',
           'scripts/audit_sensor_bag.py','scripts/sensor_recording_profile.py',
           'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py',
           'src/arena_vehicle_interface/arena_vehicle_interface/bag_contract.py',
           'tests/test_sensor_probe_inputs.py','tests/test_sensor_probe_audit.py']
paths = [ROOT/p for p in sources]+[p for p in INPUT.rglob('*') if p.is_file()]
def hashes():
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
before = hashes()
for p in sources:
    dest = HERE/'sources'/p
    dest.parent.mkdir(parents=True,exist_ok=True)
    with (ROOT/p).open('rb') as src, dest.open('xb') as out:
        shutil.copyfileobj(src,out)
def forbidden(*args,**kwargs):
    raise AssertionError('file-only inspection must not initialize ROS or create a node')
rclpy.init = rclpy.create_node = forbidden
# Comparison names are explicitly declared diagnostic expectations, not mounting evidence.
frames = {'/scan':'laser_frame','/imu/data':'imu_frame','/drive_motor/encoder':'drive_motor_encoder'}
result = analyze(INPUT,frames)
with (HERE/'inspection.json').open('x') as stream:
    json.dump(result,stream,indent=2,allow_nan=False)
topics = result['per_topic']
checks = dict(record_integrity=result['record_integrity']['record_integrity_verified'],
              five_topics_20_each=len(topics)==5 and all(r['count']==20 for r in topics.values()),
              motor_format_kept=topics['/drive_motor/encoder']['findings'].get('single_motor_format_or_nonfinite',0)==0,
              image_layout_kept=topics['/camera/color/image_raw']['findings'].get('vision_layout_unsupported',0)==0,
              uncalibrated_retained=topics['/camera/color/camera_info']['findings']['camera_info_uncalibrated']==20,
              scan_no_return_retained=topics['/scan']['scan_returns']['positive_inf']==20,
              explicit_frame_mismatch=topics['/scan']['findings']['declared_frame_mismatch']==20,
              source_and_input_unchanged=before==hashes())
report = dict(passed=all(checks.values()),checks=checks,source_input_sha256=before,
              ros_node_created=False,hardware_verified=False,controller_compatibility_verified=False,ready_to_drive=False)
with (HERE/'report.json').open('x') as stream:
    json.dump(report,stream,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='source_input_sha256'}))
raise SystemExit(0 if report['passed'] else 1)

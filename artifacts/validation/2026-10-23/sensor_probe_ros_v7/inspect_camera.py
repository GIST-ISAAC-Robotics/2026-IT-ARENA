"""이미 저장된 실패 기록을 읽기만 한다. 원본 테스트 판정/바이트를 고치지 않는다."""
import json
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from audit_sensor_probe import audit
from audit_sensor_bag import reader_for
from analyze_sensor_probe_timing import analyze

HERE = Path(__file__).resolve().parent
capture = HERE/'full_input/sensor_recording'
integrity = audit(capture)
reader = reader_for(capture, topics=json.loads((capture/'manifest.json').read_text())['topics'])
frames = []
while reader.has_next():
    topic, payload, stamp = reader.read_next()
    if topic != '/camera/color/image_raw':
        continue
    msg = deserialize_message(payload, Image)
    index = msg.data[0]
    padding = 0 if index % 2 == 0 else 16
    expected = (bytes((index, 37, 191))*1280 + b'\xee'*padding)*720
    frames.append(dict(fixture_index=index, encoding=msg.encoding, step=msg.step,
                       received_ros_ns=stamp, pixels_match=bytes(msg.data) == expected,
                       dimensions_match=(msg.width, msg.height) == (1280, 720)))
received = {f['fixture_index'] for f in frames}
report = dict(record_integrity=integrity, camera_frames=frames,
              missing_fixture_indices=sorted(set(range(20))-received),
              unique_indices=len(received),
              missing_cause_identified=False, original_sender_digests_saved=False,
              acquisition_timing_verified=False, ready_to_drive=False)
with (HERE/'camera_diagnostic.json').open('x') as stream:
    json.dump(report, stream, indent=2)
with (HERE/'timing.json').open('x') as stream:
    json.dump(analyze(capture), stream, indent=2)
print(json.dumps(dict(frames=len(frames), missing=report['missing_fixture_indices'],
                     all_pixels_match=all(f['pixels_match'] and f['dimensions_match'] for f in frames))))

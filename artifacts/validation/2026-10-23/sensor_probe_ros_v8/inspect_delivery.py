"""원래 v8 실패를 보존하고 송신/수신 경계의 끝 정렬과 누락을 따로 설명한다."""
import json
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from audit_sensor_probe import audit
from audit_sensor_bag import reader_for
from sensor_probe_delivery import DeliveryInventory
from analyze_sensor_probe_timing import analyze

HERE = Path(__file__).resolve().parent
capture = HERE/'full_input/sensor_recording'
transmissions = json.loads((HERE/'full_input/transmissions.json').read_text())
inventory = DeliveryInventory(transmissions)
integrity = audit(capture)
reader = reader_for(capture, topics=json.loads((capture/'manifest.json').read_text())['topics'])
frames = []
while reader.has_next():
    topic, payload, stamp = reader.read_next()
    inventory.observe(topic, bytes(payload))
    if topic == '/camera/color/image_raw':
        msg = deserialize_message(payload, Image)
        index = msg.data[0]
        expected = (bytes((index, 37, 191))*1280+b'\xee'*(16 if index % 2 else 0))*720
        frames.append(dict(fixture_index=index, pixels_match=bytes(msg.data) == expected))
report = dict(record_integrity=integrity, transmission_boundary=inventory.result(), camera_frames=frames,
              missing_camera_indices=sorted(set(range(20))-{f['fixture_index'] for f in frames}),
              missing_cause_identified=False, ready_to_drive=False)
with (HERE/'delivery_diagnostic.json').open('x') as stream:
    json.dump(report, stream, indent=2)
with (HERE/'timing.json').open('x') as stream:
    json.dump(analyze(capture), stream, indent=2)
print(json.dumps(dict(camera=len(frames), missing=report['missing_camera_indices'],
                     unexpected=inventory.result()['unexpected'],
                     end_alignment=inventory.result()['trailing_zero_alignment_by_topic'])))

"""3초 수신만으로 현재 MCU 프로토콜을 확인한다. UART 쓰기/펌웨어 변경 없음."""
import collections
import hashlib
import json
from pathlib import Path
import sys
import time
import zlib
import serial

sys.path.insert(0, '/home/q/it_arena_imu_preview_r161_20261112/scripts')
from probe_esp32_imu import Decoder

output = Path(sys.argv[1])
output.mkdir(exist_ok=False, parents=True)
port = serial.Serial(port=None, baudrate=921600, timeout=.1, exclusive=True)
port.dtr = False
port.rts = False
port.port = '/dev/ttyACM0'
chunks = []
try:
    port.open()
    began = time.monotonic()
    while time.monotonic()-began < 3.:
        chunks.append(port.read(max(1, min(port.in_waiting, 8192))))
finally:
    port.close()
raw = b''.join(chunks)
(output/'uart-private.bin').write_bytes(raw)
decoder = Decoder()
imu = decoder.feed(raw)
ascii_rows = []
for line in raw.splitlines():
    try:
        if line[8:9] != b' ' or int(line[:8],16) != zlib.crc32(line[9:]):
            continue
        row = json.loads(line[9:])
        ascii_rows.append({k:v for k,v in row.items() if k in (
            'event','firmware','app','boot','seq','pwm','duty','armed','active','reason',
            'count','pcnt','software_count','encoder_a','encoder_b')})
    except (ValueError, UnicodeError):
        continue
summary = dict(bytes=len(raw), raw_sha256=hashlib.sha256(raw).hexdigest(),
    transmit_bytes=0, port_closed=not port.is_open, imu_frames=len(imu),
    imu_types=dict(collections.Counter(x['type'] for x in imu)),
    imu_metadata=[x for x in imu if x['type']==3],
    imu_status=[x for x in imu if x['type']==2][-2:],
    ascii_frames=len(ascii_rows), ascii_events=dict(collections.Counter(x.get('event') for x in ascii_rows)),
    ascii_samples=ascii_rows[:4]+ascii_rows[-4:],
    limitations=['passive protocol evidence only; opening USB UART may reset board',
                 'no commands, motor/servo operation, encoder calibration, or firmware change'])
(output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary))

"""보존된 UART 바이트를 모터 벤치의 CRC/부팅 접두부 해석기로 독립 확인한다."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
from probe_esp32_motor import decode_line

raw = (ROOT / 'build/r234_uart_private.bin').read_bytes()
assert hashlib.sha256(raw).hexdigest() == '557beaa414c35c68b5ae0f5f7971decb9521252286907874465a44dfeea40719'
frames, excluded = [], []
for index, line in enumerate(raw.splitlines()):
    try:
        value, prefix = decode_line(line, allow_boot_prefix=True)
        frames.append(dict(line=index, prefix_bytes=prefix, data=value))
    except (ValueError, UnicodeError) as error:
        excluded.append(dict(line=index, bytes=len(line), reason=str(error)))
infos = [item['data'] for item in frames if item['data'].get('event') == 'info']
motors = [item['data'] for item in frames if item['data'].get('event') == 'motor']
assert len(frames) == 7 and len(infos) == 1 and infos[0]['firmware'] == 'motor_probe_v1'
assert len(motors) == 3 and all(item['active'] is False and item['armed'] is False for item in motors)
report = dict(raw_sha256=hashlib.sha256(raw).hexdigest(), crc_frames=len(frames), info=infos[0],
              motor_frames=len(motors), all_motor_inactive_unarmed=True, excluded_lines=excluded,
              boot_prefix_bytes=[item['prefix_bytes'] for item in frames if item['prefix_bytes']],
              original_summary_preserved=True,
              scope='saved bytes only; no serial open or command; raw remains Git-excluded')
(Path(__file__).parent / 'independent_audit.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
print(json.dumps(report))

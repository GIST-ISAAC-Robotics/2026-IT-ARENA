"""기존 r416 build 설정 제한 읽기. 빌드/장치/설정 변경 없음."""
import hashlib
import json
from pathlib import Path
import re

build = Path('/home/jinhyeong/.cache/it-arena/runtime-installed-r416-20261113-a')
output = Path(__file__).resolve().parent
pattern = re.compile(r'(?:CONFIG_)?(?:ESP_TASK_WDT|ESP_INT_WDT|ESP_SYSTEM_PANIC|ESP_PANIC|ESP_SYSTEM_REBOOT|ESP_SYSTEM_RESET|BOOTLOADER_WDT|ESP_BROWNOUT)')
records = []
for relative in ('sdkconfig', 'build/config/sdkconfig.h'):
    source = build/relative
    record = dict(source=str(source), exists=source.is_file())
    if source.is_file():
        raw = source.read_bytes()
        lines = [dict(line=i, text=line) for i,line in enumerate(raw.decode().splitlines(), 1)
                 if pattern.search(line)]
        record.update(sha256=hashlib.sha256(raw).hexdigest(), bytes=len(raw), selected_lines=lines)
    records.append(record)
value = dict(schema='existing_idf_watchdog_config_read_v1', build_root=str(build), files=records,
             read_only=True, build_executed=False, device_accessed=False, configuration_modified=False,
             runtime_behavior_verified=False)
with (output/'receipt.json').open('x', encoding='utf8') as stream:
    json.dump(value,stream,ensure_ascii=False,indent=2);stream.write('\n')
print(json.dumps(value,ensure_ascii=False,indent=2))

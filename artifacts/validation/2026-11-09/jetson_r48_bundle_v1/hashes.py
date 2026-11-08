"""입력 trace와 이전 r42 묶음 파일의 SHA-256을 새 파일(<before|after>)에 기록한다. python3 -I -B hashes.py before"""
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
FILES = (
    'artifacts/validation/2026-11-08/observed_corner_continuity_r48_v1/run1/follow_decisions.jsonl',
    'artifacts/validation/2026-11-08/faster_entry_consistency_r44_v1/straight_moving_lead/run1/follow_decisions.jsonl',
    'build/jetson_follow_20261108_v1.tar.gz',
    'build/jetson_follow_20261108_v3.tar.gz',
    'artifacts/validation/2026-11-08/jetson_bundle_refresh_v1/bundle_manifest.json',
    'artifacts/validation/2026-11-08/jetson_bundle_refresh_v1/v3/bundle_manifest.json',
)


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


label = sys.argv[1]
assert label in ('before', 'after')
result = {name: dict(size=(REPO/name).stat().st_size, sha256=digest(REPO/name)) for name in FILES}
with (HERE/f'input_hashes_{label}.json').open('x', encoding='utf-8') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps({k: v['sha256'][:12] for k, v in result.items()}))

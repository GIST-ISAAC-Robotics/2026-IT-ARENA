"""직전 105회귀/실행과 공통 소스 불변 확인. 새 overlay는 별도 실행기가 보존."""
import hashlib
import json
from pathlib import Path

out = Path(__file__).resolve().parent
root = out.parents[3]
before = json.loads((out.parent/'e3_harness_v1/source_manifest.json').read_text())
changed = [p for p, h in before.items() if hashlib.sha256((root/p).read_bytes()).hexdigest() != h]
print(json.dumps({'checked_files': len(before), 'changed': changed}))
if changed:
    raise SystemExit(1)

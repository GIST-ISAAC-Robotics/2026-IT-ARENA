"""등록 해시와 런타임 출처 확인이 그대로인지 확인한 뒤 기존 교통 검사기를 등록 인자 그대로 한 번 실행한다."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
r = json.loads((A/'registered.json').read_bytes())
changed = [p for p, h in r['sources'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != h]
assert not changed and not (A/'run1').exists(), changed
assert json.loads((A/'runtime_check.json').read_bytes())['ok'] is True
args = [str(A/'run1') if a == 'OUTPUT' else a for a in r['args']]
raise SystemExit(subprocess.call([sys.executable, '-B', *args], cwd=ROOT))

from pathlib import Path
import hashlib
import json
import subprocess
import sys

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
r = json.loads((A/'registered.json').read_bytes())
changed = [p for p,h in r['sources'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != h]
assert not changed and not (A/'run1').exists(), changed
args = [str(A/'run1') if a == 'OUTPUT' else a for a in r['args']]
raise SystemExit(subprocess.call([sys.executable, '-B', *args], cwd=ROOT))

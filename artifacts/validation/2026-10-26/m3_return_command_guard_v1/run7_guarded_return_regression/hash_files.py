import hashlib,json
from pathlib import Path
root=Path.cwd()
files=["src/arena_autonomy/arena_autonomy/m3_guarded_return_trial.py","tests/test_m3_guarded_return_trial.py","tests/test_m3_return_guard.py"]
print(json.dumps({p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in files},sort_keys=True,indent=2))

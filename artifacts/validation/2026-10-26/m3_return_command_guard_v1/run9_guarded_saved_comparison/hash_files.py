import hashlib,json
from pathlib import Path
root=Path.cwd(); h=root/"artifacts/validation/2026-10-26/m3_return_command_guard_v1"
files=sorted((root/"src/arena_autonomy/arena_autonomy").glob("*.py"))+[h/"compare_guarded_trial.py"]
print(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},sort_keys=True,indent=2))

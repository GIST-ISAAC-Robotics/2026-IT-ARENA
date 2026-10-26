import hashlib,json
from pathlib import Path
root=Path.cwd()
paths=sorted((root/"src/arena_autonomy/arena_autonomy").glob("*.py"))+[root/"artifacts/validation/2026-10-26/m3_return_command_guard_v1/replay_return_audit.py"]
print(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},sort_keys=True,indent=2))

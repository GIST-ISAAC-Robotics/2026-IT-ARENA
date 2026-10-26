import hashlib,json
from pathlib import Path
root=Path.cwd()
paths=sorted((root/"src/arena_autonomy/arena_autonomy").glob("*.py"))
paths += [root/p for p in ["tests/test_m3_guarded_return_trial.py","tests/test_m3_return_guard.py","tests/test_m3_baseline_release.py","tests/test_m3_retained_threats.py","tests/test_m3_release_gate_review.py","tests/test_m3_execution_trial.py","tests/test_m3_command_selection.py"]]
print(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},sort_keys=True,indent=2))

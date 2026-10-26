import hashlib,json
from pathlib import Path
root=Path.cwd(); base=root/"artifacts/validation/2026-10-25/m3_execution_review_v1"
paths=sorted((root/"src/arena_autonomy/arena_autonomy").glob("*.py"))+[root/"artifacts/validation/2026-10-26/m3_return_command_guard_v1/yield_observation_audit.py"]
for name in ("run17_observed_release_closed_loop","run21_right_encounter"):
 paths += [base/name/"run/follow_decisions.jsonl",root/"artifacts/validation/2026-10-26/m3_rear_residual_v1/cachefix_runs/run7_premises_cachefix"/(name+"__plain.json")]
print(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},sort_keys=True,indent=2))

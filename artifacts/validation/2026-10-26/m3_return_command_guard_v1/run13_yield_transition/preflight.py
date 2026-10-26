from pathlib import Path
base=Path("artifacts/validation/2026-10-25/m3_execution_review_v1")
files=["artifacts/validation/2026-10-26/m3_return_command_guard_v1/yield_transition_study.py"]
for name in ("run17_observed_release_closed_loop","run21_right_encounter"):
 files.append(str(base/name/"run/follow_decisions.jsonl"))
for name in files:
 p=Path(name); print(repr(name),"exists=",p.is_file(),"resolved=",repr(str(p.resolve())))
 if not p.is_file(): raise SystemExit(92)

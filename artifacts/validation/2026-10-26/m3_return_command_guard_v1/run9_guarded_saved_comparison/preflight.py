from pathlib import Path
files=["artifacts/validation/2026-10-26/m3_return_command_guard_v1/compare_guarded_trial.py"]
for x in files:
 p=Path(x); print(repr(x),"exists=",p.is_file(),"resolved=",repr(str(p.resolve())))
 if not p.is_file(): raise SystemExit(92)

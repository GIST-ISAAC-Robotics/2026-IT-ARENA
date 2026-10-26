from pathlib import Path
files=["src/arena_autonomy/arena_autonomy/m3_guarded_return_trial.py","tests/test_m3_guarded_return_trial.py","tests/test_m3_return_guard.py"]
for name in files:
 p=Path(name)
 print(repr(name),"exists=",p.is_file(),"resolved=",repr(str(p.resolve())))
 if not p.is_file(): raise SystemExit(92)

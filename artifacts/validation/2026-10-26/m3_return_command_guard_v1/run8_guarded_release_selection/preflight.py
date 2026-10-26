from pathlib import Path
files=["tests/test_m3_guarded_return_trial.py","tests/test_m3_return_guard.py","tests/test_m3_baseline_release.py","tests/test_m3_retained_threats.py","tests/test_m3_release_gate_review.py","tests/test_m3_execution_trial.py","tests/test_m3_command_selection.py"]
for name in files:
 p=Path(name)
 print(repr(name),"exists=",p.is_file(),"resolved=",repr(str(p.resolve())))
 if not p.is_file(): raise SystemExit(92)

from pathlib import Path
files=["scripts/m4_second_start.py","scripts/m4_operation_trial.py","scripts/m4_operation_eval.py","scripts/validate_g1_traffic.py","tests/test_m4_second_start.py","tests/test_m4_operation_trial.py","tests/test_m4_operation_cli.py"]
for name in files:
 p=Path(name); print(repr(name),"exists=",p.is_file(),"resolved=",repr(str(p.resolve())))
 if not p.is_file(): raise SystemExit(92)

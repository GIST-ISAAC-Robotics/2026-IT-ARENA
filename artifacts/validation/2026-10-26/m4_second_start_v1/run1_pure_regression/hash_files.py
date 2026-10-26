import hashlib,json
from pathlib import Path
root=Path.cwd()
files=["scripts/m4_second_start.py","scripts/m4_operation_trial.py","scripts/m4_operation_eval.py","scripts/validate_g1_traffic.py","tests/test_m4_second_start.py","tests/test_m4_operation_trial.py","tests/test_m4_operation_cli.py"]
print(json.dumps({p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in files},sort_keys=True,indent=2))

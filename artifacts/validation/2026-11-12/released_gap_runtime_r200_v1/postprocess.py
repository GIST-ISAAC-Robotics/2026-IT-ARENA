"""시험 종료 후 고유 파티션 잔여 확인과 기존 평가기 실행. 로컬 WSL 전용."""
import json
from pathlib import Path
import subprocess
import sys

A = Path(__file__).resolve().parent
D = A/'two_actor_run1'
ROOT = A.parents[3]
report = json.loads((D/'run1/report.json').read_bytes())
partition = report['gazebo_stop_service']['partition']
remaining = []
for process in Path('/proc').iterdir():
    if not process.name.isdecimal():
        continue
    try:
        values = (process/'environ').read_bytes().split(b'\0')
        if ('GZ_PARTITION='+partition).encode() in values:
            remaining.append(process.name)
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
with (D/'postflight_processes.txt').open('x') as out:
    out.write('\n'.join(remaining))
print(json.dumps(dict(partition=partition, remaining=remaining)), flush=True)
subprocess.run([sys.executable, str(ROOT/'scripts/surface_contact_eval.py'), str(D/'run1'),
                '--output', str(D/'surface.json')], check=True)
raise SystemExit(subprocess.call([sys.executable,str(A/'assess.py')]))

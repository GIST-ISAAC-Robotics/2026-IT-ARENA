#!/usr/bin/env bash
set -eo pipefail
/bin/true
python3 -B - <<'PY'
from pathlib import Path
import json,os
rows=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) in (os.getpid(),os.getppid()):continue
    try:
        argv=[v.decode(errors='replace') for v in (p/'cmdline').read_bytes().split(b'\0') if v]
        if any('guard_crawl_drive_r452_v1' in v for v in argv):rows.append(dict(pid=int(p.name),argv=argv))
    except (FileNotFoundError,PermissionError,ProcessLookupError):pass
print(json.dumps(dict(bin_true_completed=True,remaining_owned=rows,simulation_started=False)))
assert not rows,rows
PY

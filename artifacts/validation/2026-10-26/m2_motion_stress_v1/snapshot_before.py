from pathlib import Path
import hashlib
import json
import zipfile

root=Path.cwd()
out=Path(__file__).resolve().parent
paths=['scripts/validate_g1_traffic.py','scripts/motion_fault_audit.py']
raw={p:(root/p).read_bytes() for p in paths}
with zipfile.ZipFile(out/'before_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p,b in raw.items(): z.writestr(p,b)
(out/'before_sha256.json').write_text(json.dumps({p:hashlib.sha256(b).hexdigest() for p,b in raw.items()},indent=2))

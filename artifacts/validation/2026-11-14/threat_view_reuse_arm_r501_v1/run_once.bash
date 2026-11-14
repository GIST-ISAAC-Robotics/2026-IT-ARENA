#!/usr/bin/env bash
# root가 Jetson 소유권을 r501에 넘기고 ZIP 전송을 끝낸 뒤 한 번만 실행한다.
set -euo pipefail
python3 - <<'PY'
import hashlib
from pathlib import Path
import zipfile
archive = Path('/home/q/it-arena-r501-a.zip')
root = Path('/home/q/it-arena-r501-a')
assert hashlib.sha256(archive.read_bytes()).hexdigest() == 'cb7a32f0d65a6646ecf3f779df227defcee21661dc0116979fad39b2dd95bc47'
assert not root.exists() and not root.with_name(root.name+'-results.zip').exists()
with zipfile.ZipFile(archive) as source:
    for name in source.namelist():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts and '\\' not in name
    source.extractall(root)
assert hashlib.sha256((root/'source_manifest.json').read_bytes()).hexdigest() == '517b39b26285227f51dddb5929691fd13d81c7754c576a79197ac4bebae3d5da'
PY
cd /home/q/it-arena-r501-a
python3 -B execute.py --resource-owner r501

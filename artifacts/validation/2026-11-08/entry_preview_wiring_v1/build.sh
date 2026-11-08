#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-08/entry_preview_wiring_v1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select arena_autonomy arena_bringup > "$A/build.log" 2>&1
source install/local_setup.bash
python3 -B - "$A" > "$A/installed_check.json" <<'PY'
import hashlib,importlib,json,sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
a=Path(sys.argv[1]);m=json.loads((a/'wiring_manifest.json').read_bytes())
results={}
for name,want in {**m['candidate_overlay']['after'],**m['after']}.items():
    if '/arena_autonomy/' in name:
        path=Path(importlib.import_module('arena_autonomy.'+Path(name).stem).__file__)
    else:
        path=Path(get_package_share_directory('arena_bringup'))/'launch/simulation.launch.py'
    got=hashlib.sha256(path.read_bytes()).hexdigest()
    assert got==want,(name,path,got,want)
    results[name]=dict(loaded_path=str(path),sha256=got)
print(json.dumps(results,indent=2))
PY
printf '0\n' > "$A/build.exit"
tail -n 6 "$A/build.log"

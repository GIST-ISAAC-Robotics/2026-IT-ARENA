#!/usr/bin/env bash
# 재생 전용 격리 사본: solo_straight_closed_loop_v1 등록 해시와 같은 arena_autonomy·arena_vehicle_interface만 복사·검증.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
REG=artifacts/validation/2026-11-05/solo_straight_closed_loop_v1/registered.json
S="$HOME/arena_scratch/corner_continuity/src"
if [ ! -d "$S" ]; then
  mkdir -p "$S"
  for pkg in arena_autonomy/arena_autonomy arena_vehicle_interface/arena_vehicle_interface; do
    mkdir -p "$S/$(dirname "$pkg")"; cp -r "src/$pkg" "$S/$pkg"
  done
fi
find "$S" -name __pycache__ -prune -exec rm -rf {} +
python3 -I -B - "$S" "$REG" <<'PY'
import hashlib, json, sys
from pathlib import Path
root = Path(sys.argv[1]); reg = json.loads(Path(sys.argv[2]).read_bytes())['input_sha256']
files = sorted(p for p in root.rglob('*') if p.is_file())
bad = ['src/'+p.relative_to(root).as_posix() for p in files
       if reg.get('src/'+p.relative_to(root).as_posix()) != hashlib.sha256(p.read_bytes()).hexdigest()]
print(json.dumps({'copied': len(files), 'not_matching_registration': bad}))
raise SystemExit(1 if bad or not files else 0)
PY

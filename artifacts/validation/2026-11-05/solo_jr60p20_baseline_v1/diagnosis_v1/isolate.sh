#!/usr/bin/env bash
# 재생 전용 격리 사본. run1 등록 해시와 같은 파일만 ~/arena_scratch에 복사한다(작업트리 변경과 분리).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.."
A=artifacts/validation/2026-11-05/solo_jr60p20_baseline_v1
S="$HOME/arena_scratch/solo_diag_v1_src"
# 이미 있으면 다시 복사하지 않고 검증만 한다(작업트리가 등록 뒤 바뀌어도 사본은 등록 상태 유지).
# 사본이 없고 작업트리도 바뀌었으면 HEAD 01d9e11c + worktree_diff.patch + registered_untracked_sources.zip으로 복원한다.
if [ ! -d "$S" ]; then
  mkdir -p "$S"
  for pkg in arena_autonomy/arena_autonomy arena_vehicle_interface/arena_vehicle_interface; do
    mkdir -p "$S/$(dirname "$pkg")"
    cp -r "src/$pkg" "$S/$pkg"
  done
fi
find "$S" -name __pycache__ -prune -exec rm -rf {} +
# 사본을 등록 해시와 대조한다. 등록과 다른 파일이 있으면 실패.
python3 -I -B - "$S" "$A/registered.json" <<'PY'
import hashlib, json, sys
from pathlib import Path
root = Path(sys.argv[1]); reg = json.loads(Path(sys.argv[2]).read_bytes())['input_sha256']
files = sorted(p for p in root.rglob('*') if p.is_file())
bad = []
for p in files:
    rel = 'src/' + p.relative_to(root).as_posix()
    if reg.get(rel) != hashlib.sha256(p.read_bytes()).hexdigest():
        bad.append(rel)
print(json.dumps({'copied': len(files), 'not_matching_registration': bad}))
raise SystemExit(1 if bad or not files else 0)
PY

#!/usr/bin/env bash
# 마감 검사: 전체 회귀, 등록 동결 해시, 플러그인 바이너리, 원시 Git 제외·미추적, 실행 소스 해시, 잔존 프로세스.
source "$(dirname "$0")/env.sh"
tag="${1:?tag}"; test ! -e "$base/pytest_$tag.log" || exit 2
python3 -B -m pytest -q -p no:cacheprovider tests > "$base/pytest_$tag.log" 2>&1; echo "pytest rc=$?"; tail -2 "$base/pytest_$tag.log"
for f in pre_run_freeze.sha256 addendum_v2_freeze.sha256 addendum_c3_freeze.sha256; do
  echo "== $f"; sha256sum -c "$base/$f" 2>&1 | grep -v ': OK$' || echo "all OK"; done
sha256sum -c "$base/plugin_v1.sha256"
for f in $(ls -d $base/*/ ); do for raw in braking_diagnostics wheel_contacts requested_commands safe_commands actuation_status output_status; do
  p="$f$raw.jsonl"; [ -e "$p" ] || continue
  git check-ignore -q "$p" || echo "NOT_IGNORED $p"
  git ls-files --error-unmatch "$p" >/dev/null 2>&1 && echo "TRACKED $p"
done; p="${f}evaluator_timing.i64"; [ -e "$p" ] && { git check-ignore -q "$p" || echo "NOT_IGNORED $p"; }; done; echo "raw policy checked"
python3 - <<'PY'
import json, pathlib, hashlib
base = pathlib.Path('artifacts/validation/2026-10-09/command_apply_trace_v1')
r2 = json.loads(pathlib.Path('config/tests/scheduled_command_r2_sha256.json').read_text())
want = {**{k: v['after'] for k, v in r2['approved_evaluation_scripts'].items()}, **r2['sha256'], **r2['implementation_sha256']}
for run in ('a1_base', 'a2_inject_late', 'c1_trace', 'c2_trace', 'c3_trace'):
    rep = json.loads((base/run/'report.json').read_text())
    src = rep['source_sha256']
    bad = [k for k, v in want.items() if k in src and src[k] != v]
    missing = [k for k in want if k not in src]
    print(run, 'source_matches_r2', not bad, 'missing', missing, 'plugin', rep['plugin_binary_sha256'][:12], 'unchanged', rep['source_unchanged'])
PY
ps -eo pid,args | grep -E 'gz sim|probe_braking_contact|parameter_bridge|rotating_lidar|actuation_ros|lidar_safety|gz topic -e' | grep -v grep || echo "NO_RESIDUAL"

#!/usr/bin/env bash
# 마감 검사: 전체 회귀, 플러그인 .so 불변, 원시 고빈도 자료 Git 제외·미추적, 실행 소스 불변, 잔존 프로세스.
set +u
source "$(dirname "$0")/env.sh"
tag="${1:?tag}"; test ! -e "$base/pytest_$tag.log"
python3 -B -m pytest -q -p no:cacheprovider tests > "$base/pytest_$tag.log" 2>&1; echo "pytest rc=$?"; tail -2 "$base/pytest_$tag.log"
sha256sum -c "$base/plugin_before.txt" > "$base/plugin_after_$tag.txt" 2>&1; cat "$base/plugin_after_$tag.txt"
sha256sum -c "$base/pre_run_freeze.sha256" 2>&1 | grep -v ': OK$' || echo "freeze: all OK"
for f in $(ls -d $base/*/ ); do for raw in braking_diagnostics wheel_contacts requested_commands safe_commands actuation_status output_status; do
  p="$f$raw.jsonl"; [ -e "$p" ] || continue
  git check-ignore -q "$p" || echo "NOT_IGNORED $p"
  git ls-files --error-unmatch "$p" >/dev/null 2>&1 && echo "TRACKED $p"
done; done; echo "raw policy checked"
ps -eo pid,cmd | grep -E 'gz sim|probe_braking_contact|parameter_bridge|rotating_lidar|actuation_ros|lidar_safety' | grep -v grep || echo "no residual"

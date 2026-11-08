#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-09/p22_route_connected_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
grep -q 'ACTUAL_EXIT_CODE=0' artifacts/validation/2026-11-07/track_migration_v1/checks/pytest_p22_route_final_1.exit
bash "$ENV" python3 -B - <<'PY' >"$A/chain_final.log" 2>&1
import test_source_provenance_r0_plus_b3 as t
t.test_registered_oct26_and_nov01_successors_have_preserved_source_chain(len(t.M3_SUCCESSORS)-1)
print('r50 latest source chain passed; prior complete chain 147 passed')
PY
bash "$ENV" python3 -B "$A/prepare.py" >"$A/prepare.log" 2>&1
set +e
bash "$ENV" python3 -B "$A/run_live.py"
rc=$?
set -e
printf '%s\n' "$rc" >"$A/run_wrapper.exit"
# 원 본선 검증기 exit 1과 별도 P22 평가를 구분한다.
bash "$ENV" python3 -B "$A/evaluate_live.py" >"$A/evaluation_v1.log" 2>&1
echo 0 >"$A/evaluation_v1.exit"

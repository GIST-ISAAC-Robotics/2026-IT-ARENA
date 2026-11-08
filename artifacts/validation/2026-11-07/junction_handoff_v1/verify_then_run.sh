#!/usr/bin/env bash
# WSL 시작 성공 후 같은 연결 안에서 회귀→등록→단독1회를 순차 진행한다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-07/junction_handoff_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [[ -e "$A/registered.json" || -e "$A/run1" ]]; then echo 'already registered/run; refuse';exit 3;fi
bash artifacts/validation/2026-11-07/track_migration_v1/run_tests.sh standing_reacquire_r11 tests/test_standing_wall_reacquire.py tests/test_guard_curve_trial.py tests/test_g1_solo_candidate_validation.py tests/test_source_provenance_r0_plus_b3.py::test_registered_oct26_and_nov01_successors_have_preserved_source_chain
unit=$?
echo "UNIT_EXIT=$unit" > "$A/verify_then_run.exit"
if [[ $unit != 0 ]]; then exit "$unit";fi
bash "$ENV" python3 -B "$A/register.py" > "$A/register.log" 2>&1
registration=$?
echo "REGISTRATION_EXIT=$registration" >> "$A/verify_then_run.exit"
if [[ $registration != 0 ]]; then exit "$registration";fi
bash "$A/run.sh"
run=$?
echo "RUN_EXIT=$run" >> "$A/verify_then_run.exit"
if [[ -f "$A/run1/report.json" ]]; then
  bash "$ENV" python3 -B scripts/surface_contact_eval.py "$A/run1" --output "$A/surface_run1.json" > "$A/surface_run1.log" 2>&1
  echo "SURFACE_EXIT=$?" >> "$A/verify_then_run.exit"
fi
exit "$run"

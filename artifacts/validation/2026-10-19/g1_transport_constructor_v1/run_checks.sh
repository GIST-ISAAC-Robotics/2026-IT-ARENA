#!/usr/bin/env bash
set -u
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local' || exit 2
export PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts
evidence=artifacts/validation/2026-10-19/g1_transport_constructor_v1
if [[ -e "$evidence/pytest.log" || -e "$evidence/run" ]]; then
    printf 'Fresh evidence path required\n' >&2
    exit 2
fi
python3 -m pytest tests/test_scene_transport_candidate.py -q > "$evidence/pytest.log" 2>&1
test_rc=$?
printf '\nPYTEST_EXIT_CODE=%s\n' "$test_rc" >> "$evidence/pytest.log"
tail -n 25 "$evidence/pytest.log"
if [[ "$test_rc" -ne 0 ]]; then exit "$test_rc"; fi
python3 scripts/benchmark_scene_transport.py \
    artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl \
    --output "$evidence/run" > "$evidence/benchmark.log" 2>&1
benchmark_rc=$?
printf '\nBENCHMARK_EXIT_CODE=%s\n' "$benchmark_rc" >> "$evidence/benchmark.log"
tail -n 15 "$evidence/benchmark.log"
exit "$benchmark_rc"

#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-21/g1_origin_contract_v1
if [[ -e "$evidence/pytest.log" || -e "$evidence/profile_on" ]]; then
    printf 'Fresh evidence path required\n' >&2
    exit 2
fi
find src scripts tests config -type f \( -name '*.py' -o -name '*.json' -o -name '*.yaml' \) -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_before.sha256"
set +e
python3 -B -m pytest -q -p no:cacheprovider tests > "$evidence/pytest.log" 2>&1
test_rc=$?
printf '\nPYTEST_EXIT_CODE=%s\n' "$test_rc" >> "$evidence/pytest.log"
tail -n 30 "$evidence/pytest.log"
if [[ "$test_rc" -ne 0 ]]; then exit "$test_rc"; fi
python3 scripts/profile_follow_trace.py \
    artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl \
    --output "$evidence/profile_on" --input-mode stream --repeats 1 --warmups 0 > "$evidence/profile_on.log" 2>&1
on_rc=$?
printf '\nPROFILE_ON_EXIT_CODE=%s\n' "$on_rc" >> "$evidence/profile_on.log"
if [[ "$on_rc" -ne 0 ]]; then tail -n 15 "$evidence/profile_on.log"; exit "$on_rc"; fi
python3 scripts/profile_follow_trace.py \
    artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl \
    --output "$evidence/profile_off" --input-mode stream --repeats 1 --warmups 0 > "$evidence/profile_off.log" 2>&1
off_rc=$?
printf '\nPROFILE_OFF_EXIT_CODE=%s\n' "$off_rc" >> "$evidence/profile_off.log"
find src scripts tests config -type f \( -name '*.py' -o -name '*.json' -o -name '*.yaml' \) -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_after.sha256"
diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_hash_diff.txt"
hash_rc=$?
printf 'PROFILE_ON_EXIT_CODE=%s\nPROFILE_OFF_EXIT_CODE=%s\nSOURCE_HASH_DIFF_EXIT_CODE=%s\n' "$on_rc" "$off_rc" "$hash_rc"
if [[ "$off_rc" -ne 0 || "$hash_rc" -ne 0 ]]; then exit 1; fi
exit 0

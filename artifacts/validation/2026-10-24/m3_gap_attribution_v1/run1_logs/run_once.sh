#!/usr/bin/env bash
set -u
BASE="$(cd -- "$(dirname -- "$0")" && pwd)"
LABEL="${1:?new run name required}"
[[ "$LABEL" =~ ^run[0-9]+$ ]] || exit 90
OUT="$BASE/$LABEL"
[[ ! -e "$OUT" ]] || exit 91
LOG="$BASE/${LABEL}_logs"
mkdir "$LOG" || exit 92
cp "$BASE/analyze.py" "$BASE/test_attribution.py" "$BASE/run_once.sh" "$LOG/" || exit 93
sha256sum "$BASE/analyze.py" "$BASE/test_attribution.py" "$BASE/run_once.sh" > "$LOG/source_before.txt"
PYTHONPATH="$BASE${PYTHONPATH:+:$PYTHONPATH}" python3 -m pytest -q -p no:cacheprovider "$BASE/test_attribution.py" > "$LOG/pytest.log" 2>&1
test_code=$?
printf '%s\n' "$test_code" > "$LOG/pytest_exit.txt"
if [ "$test_code" -ne 0 ]; then exit "$test_code"; fi
python3 "$BASE/analyze.py" "$OUT" > "$LOG/stdout.json" 2> "$LOG/stderr.log"
code=$?
printf '%s\n' "$code" > "$LOG/analysis_exit.txt"
sha256sum -c "$LOG/source_before.txt" > "$LOG/source_after.txt" 2>&1
hash_code=$?
printf '%s\n' "$hash_code" > "$LOG/hash_exit.txt"
if [ "$code" -ne 0 ]; then exit "$code"; fi
exit "$hash_code"

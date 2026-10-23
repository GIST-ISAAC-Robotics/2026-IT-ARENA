#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
out=artifacts/validation/2026-10-23/imu_source_contract_v1
test ! -e "$out/checks_exit.txt"
finish() { result=$?; printf '%s\n' "$result" > "$out/checks_exit.txt"; }
trap finish EXIT
sha256sum "$out"/upstream/* "$out/probe.c" "$out/run_checks.sh" > "$out/sources_before.sha256"
cc -std=c11 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -no-pie -Wall -Wextra \
  -I"$out/upstream" "$out/probe.c" "$out/upstream/lsm6dsox_reg.c" -lm -o "$out/probe" \
  > "$out/compile.stdout" 2> "$out/compile.stderr"
ASAN_OPTIONS=detect_leaks=1 "$out/probe" > "$out/report.json" 2> "$out/probe.stderr"
sha256sum "$out"/upstream/* "$out/probe.c" "$out/run_checks.sh" > "$out/sources_after.sha256"
diff -u "$out/sources_before.sha256" "$out/sources_after.sha256" > "$out/source_hash_diff.txt"

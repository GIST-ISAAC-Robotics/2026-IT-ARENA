#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/traffic_faster_reacquire_v1
E=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
for label in base candidate; do
  if [ -e "$A/replay_${label}_result.json" ]; then exit 3; fi
  bash "$E" python3 -B "$A/replay.py" "$label" > "$A/replay_${label}.log" 2>&1
  status=$?; printf '%s
' "$status" > "$A/replay_${label}.exit"
  [ "$status" -eq 0 ] || exit "$status"
done

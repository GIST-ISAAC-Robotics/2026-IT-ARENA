#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-07/late_start_marker_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [[ -e "$A/registered_v2.json" || -e "$A/occlusion_run2" ]]; then echo 'already registered/run; refuse';exit 3;fi
bash "$ENV" python3 -B "$A/register_v2.py" > "$A/register_v2.log" 2>&1 || exit $?
bash "$ENV" python3 -B "$A/occlusion_eval_v2.py" --output "$A/occlusion_run2" --track official_v2026_11_06 --local-sensor-config src/arena_description/config/b0390_c1_jr60p20_sim.yaml --wall-limit-s 600 > "$A/occlusion_run2.log" 2>&1
code=$?
echo "$code" > "$A/occlusion_run2.exit"
python3 - "$A" <<'VERIFY'
import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1]);d=json.loads((p/'registered_v2.json').read_text());changed=[f for f,h in d['input_sha256'].items() if hashlib.sha256(Path(f).read_bytes()).hexdigest()!=h]
(p/'unchanged_v2.json').write_text(json.dumps(dict(count=len(d['input_sha256']),changed=changed),indent=2)+'\n')
print('source changes',changed)
VERIFY
exit "$code"

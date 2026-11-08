"""assess.py 함수를 기존 결과에 읽기 전용으로 적용한다: r27 run1(무주입, A 경로)과 run57(m3 등록 C1 주입, B 경로)."""
import json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]; ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE)); import assess
read = assess.read
out = {}
r27 = ROOT/'artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1/two_actor_course_lap'
run = r27/'run1'; r = read(run/'report.json'); acq = read(run/'acquisition.json'); rec = assess.jsonl(run/'scans_received.jsonl')
assess.HERE = r27.parent  # r27_gates의 registered.json 대신 r27 등록본으로 원 12조건 재현 확인
orig = assess.read
def patched(p):
    p = Path(p)
    if p.name == 'registered.json' and p.parent == r27.parent:
        return orig(r27/'registered.json')
    if p.name == 'postflight_integrity_run1.json':
        d = orig(p); d.setdefault('snapshot_unchanged', True); return d
    return orig(p)
assess.read = patched
g, s = assess.r27_gates(r27, run, r, read(r27/'surface_run1.json'))
out['r27_12_gates'] = g; out['r27_12_match_original'] = g == orig(r27/'assessment_run1.json')['gates']
out['r27_as_A_injection'] = assess.injection_gates('A', run, r, acq, rec)[0]
out['r27_decisions'] = assess.decision_gates(run, acq)
m = ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1/run57_faster_lead_sensor_stress/run'
rm = orig(m/'report.json'); am = orig(m/'acquisition.json'); rcv = assess.jsonl(m/'scans_received.jsonl')
out['run57_as_B_injection'] = assess.injection_gates('B', m, rm, am, rcv)
out['run57_as_A_must_fail'] = assess.injection_gates('A', m, rm, am, rcv)[0]
out['run57_decisions'] = assess.decision_gates(m, am)
print(json.dumps(out, indent=1, default=str))

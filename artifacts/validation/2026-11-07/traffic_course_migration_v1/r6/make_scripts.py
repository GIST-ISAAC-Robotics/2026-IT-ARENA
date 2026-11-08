"""r6 실행용 run.py/run.sh/assess.py를 상위 폴더 원본에서 경로·실행 이름만 바꿔 생성한다(판정 논리 동일).

출력 폴더는 r6/<case>/run2 (사용자 지시 'run2'). 상위 slow_lead/run1·등록은 그대로 둔다.
"""
from pathlib import Path

HERE = Path(__file__).resolve().parent
PARENT = HERE.parent
EDITS = {
    'run.py': [("ROOT = HERE.parents[3]", "ROOT = HERE.parents[4]"),
               ("output = folder/'run1'", "output = folder/'run2'")],
    'assess.py': [("ROOT = HERE.parents[3]", "ROOT = HERE.parents[4]"),
                  ("run = folder/'run1'", "run = folder/'run2'"),
                  ("course = read(HERE/'new_track_course.json')", "course = read(HERE.parent/'new_track_course.json')"),
                  ("surface = read(folder/'surface.json')", "surface = read(folder/'surface_run2.json')"),
                  ("post = read(folder/'postflight_integrity.json')", "post = read(folder/'postflight_integrity_run2.json')"),
                  ("leftover = (folder/'postflight_processes.txt')", "leftover = (folder/'postflight_processes_run2.txt')"),
                  ("(folder/'assessment.json')", "(folder/'assessment_run2.json')")],
    'run.sh': [("A=artifacts/validation/2026-11-07/traffic_course_migration_v1\n",
                "A=artifacts/validation/2026-11-07/traffic_course_migration_v1/r6\n"),
               ('cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."', 'cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.."'),
               ('-e "$D/run1" || -e "$D/run1.exit"', '-e "$D/run2" || -e "$D/run2.exit"'),
               ('> "$D/preflight_processes.txt"', '> "$D/preflight_processes_run2.txt"'),
               ('echo "PREFLIGHT_BUSY=1" > "$D/run1.exit"', 'echo "PREFLIGHT_BUSY=1" > "$D/run2.exit"'),
               ('> "$D/run1_console.log"', '> "$D/run2_console.log"'),
               ('if [[ -f "$D/run1/report.json" ]]; then\n  bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$D/run1" --output "$D/surface.json" > "$D/surface.log"',
                'if [[ -f "$D/run2/report.json" ]]; then\n  bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$D/run2" --output "$D/surface_run2.json" > "$D/surface_run2.log"'),
               ("(d/'postflight_integrity.json')", "(d/'postflight_integrity_run2.json')"),
               ('> "$D/postflight_processes.txt"', '> "$D/postflight_processes_run2.txt"'),
               ('if [[ -f "$D/run1/report.json" ]]; then\n  bash "$ENV" /usr/bin/python3 -B "$A/assess.py" "$N" > "$D/assessment.log"',
                'if [[ -f "$D/run2/report.json" ]]; then\n  bash "$ENV" /usr/bin/python3 -B "$A/assess.py" "$N" > "$D/assessment_run2.log"'),
               ('> "$D/run1.exit"\nexit', '> "$D/run2.exit"\nexit')],
}
for name, pairs in EDITS.items():
    text = (PARENT/name).read_text(encoding='utf-8')
    for old, new in pairs:
        assert text.count(old) == 1, (name, old)
        text = text.replace(old, new)
    target = HERE/name
    assert not target.exists()
    target.write_bytes(text.encode('utf-8'))
print('generated', list(EDITS))

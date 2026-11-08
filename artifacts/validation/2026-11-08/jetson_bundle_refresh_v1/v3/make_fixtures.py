"""CLI 최소 검사용 파생 trace: 원 기록의 header와 앞 100판단(사이 reset 포함)에 맞춘 footer.

판단은 이전 입력만으로 결정되므로 앞부분 재생 출력은 원 기록과 같아야 한다. 원 기록은 읽기만 한다.
주행 결과나 전체 기록 재생을 대신하지 않는다.
"""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
CALLS = 100
SOURCES = {
    'r25_solo_first100.jsonl': 'artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/run1/follow_decisions.jsonl',
    'r41_handoff_first100.jsonl': 'artifacts/validation/2026-11-08/current_corner_support_v1/run1/follow_decisions.jsonl',
}
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
(HERE/'fixtures').mkdir(exist_ok=False)
record = {}
for name, source in SOURCES.items():
    lines, calls = [], 0
    original = sha(REPO/source)
    with (REPO/source).open(encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            assert event['seq'] == len(lines) and event['kind'] != 'footer'
            lines.append(line if line.endswith('\n') else line+'\n')
            calls += event['kind'] == 'call'
            if calls == CALLS:
                break
    # footer 계수는 footer seq(=header 포함 앞 event 수)와 같다. v2는 1 적게 써서 CLI가 정상 거절했다.
    footer = dict(accepted=len(lines), written=len(lines), dropped=0, failed=0, pending=0,
                  worker_alive=False, capture_errors=0)
    lines.append(json.dumps(dict(seq=len(lines), kind='footer', data=footer))+'\n')
    target = HERE/'fixtures'/name
    with target.open('x', encoding='utf-8', newline='\n') as stream:
        stream.writelines(lines)
    assert sha(REPO/source) == original
    record[name] = dict(source=source, source_sha256=original, events=len(lines), calls=calls, sha256=sha(target),
                        derived='header + first calls/resets of source + regenerated footer')
with (HERE/'fixtures.json').open('x', encoding='utf-8') as stream:
    json.dump(record, stream, ensure_ascii=False, indent=2)
print(json.dumps(record, ensure_ascii=False))

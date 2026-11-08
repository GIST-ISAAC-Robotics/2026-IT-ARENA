"""r30 직후 소스와 새 소스의 동일 입력 비교. 오래된 기대 출력의 기존 차이를 분리한다."""
import hashlib
import json
import shutil
import sys
import tempfile
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
MODE = sys.argv[1]
assert MODE in ('before', 'current')
if MODE == 'before':
    tree = Path(tempfile.mkdtemp(prefix='arena_before_faster_'))
    shutil.copytree(ROOT/'src/arena_autonomy/arena_autonomy', tree/'arena_autonomy',
                    ignore=shutil.ignore_patterns('__pycache__'))
    with zipfile.ZipFile(HERE/'before.zip') as archive:
        for name in archive.namelist():
            prefix = 'src/arena_autonomy/arena_autonomy/'
            if name.startswith(prefix):
                (tree/'arena_autonomy'/name[len(prefix):]).write_bytes(archive.read(name))
    sys.path.insert(0, str(tree))

from arena_autonomy.follow_trace import replay_events, equal_output, comparable_decision

sources = {
    'slow': ROOT/'artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/run1/follow_decisions.jsonl',
    'medium': HERE.parent/'medium_pass_connected_v1/straight_moving_lead/run1/follow_decisions.jsonl',
}
summaries = {}
for label, path in sources.items():
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    with path.open() as stream:
        result = replay_events(json.loads(line) for line in stream if line.strip())
    outcomes = result.pop('outcomes')
    summary = {k: v for k, v in result.items() if k != 'mismatches'}
    summary.update(original_trace_mismatches=len(result['mismatches']), source_sha256=digest)
    with (HERE/f'{MODE}_{label}_rows.jsonl').open('w') as stream:
        for row in outcomes:
            stream.write(json.dumps(row, ensure_ascii=False)+'\n')
    if MODE == 'current':
        with (HERE/f'before_{label}_rows.jsonl').open() as stream:
            old = [json.loads(line) for line in stream]
        assert len(old) == len(outcomes)
        mismatches = []
        def decision(row):
            return [row['speed_mps'], row['steering_rad'], row['details']]
        for a, b in zip(old, outcomes):
            if a['seq'] != b['seq'] or not equal_output(comparable_decision(decision(a)), comparable_decision(decision(b))):
                mismatches.append({'before': a, 'current': b})
        summary['change_from_before'] = len(mismatches)
        if mismatches:
            (HERE/f'current_{label}_differences.json').write_text(json.dumps(mismatches[:3], ensure_ascii=False, indent=2)+'\n')
    summaries[label] = summary
    assert digest == hashlib.sha256(path.read_bytes()).hexdigest()
    print(label, json.dumps(summary, ensure_ascii=False), flush=True)
(HERE/f'{MODE}_comparison_summary.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2)+'\n')
if MODE == 'current':
    assert all(s['change_from_before'] == 0 for s in summaries.values())

"""이번 변경의 소스/주행/정지/종료를 구분해 확인합니다. 원래 판정은 수정하지 않습니다."""
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
RUNS = ('before_g1_slow_s0', 'after_g1_slow_s0', 'after_r0_stopped_s0',
        'after_r0_stopped_s0_repeat', 'after_g1_stopped_s0')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


results = []
for name in RUNS:
    folder = HERE / name
    report = json.loads((folder / 'report.json').read_text())
    assert report['run_valid'] and report['shutdown_clean'] and report['source_unchanged'], name
    assert all(report['stop_checks'].values()), (name, report['stop_checks'])
    assert report['evaluation']['gap']['actor_contact_messages_with_ego'] == 0, name
    with ZipFile(folder / 'source_snapshot.zip') as archive:
        assert all(hashlib.sha256(archive.read(path)).hexdigest() == digest
                   for path, digest in report['source_sha256'].items()), name
    for raw in ('ego_poses', 'actor_poses', 'actor_contacts', 'scans_received', 'autonomy_status'):
        assert sha(folder / f'{raw}.jsonl') == report[f'{raw}_sha256'], (name, raw)
    if report['mode'] == 'g1':
        assert report['passed'] and report['behavior_gates_passed'], name
        assert json.loads((folder / 'postcoverage.json').read_text())['passed'], name
    else:
        assert report['r0_meets_g1_gates'] is False, name
        raw = folder / 'braking_diagnostics.jsonl'
        assert sha(raw) == report['braking_diagnostics']['sha256'], name
    results.append({
        'run': name, 'report_sha256': sha(folder / 'report.json'),
        'source_zip_sha256': sha(folder / 'source_snapshot.zip'),
        'run_valid': report['run_valid'], 'behavior_gates_passed': report['behavior_gates_passed'],
        'shutdown_clean': report['shutdown_clean'],
        'gap': report['evaluation']['gap'], 'case_metrics': report['evaluation']['case_metrics'],
    })
with ZipFile(HERE / RUNS[0] / 'source_snapshot.zip') as before, ZipFile(HERE / RUNS[1] / 'source_snapshot.zip') as after:
    differences = [name for name in before.namelist() if before.read(name) != after.read(name)]
    assert set(differences) == {'src/arena_gazebo/worlds/it_arena_official/world.sdf',
                                'src/arena_gazebo/worlds/it_arena_official/scene.json'}
latest = json.loads((HERE / RUNS[-1] / 'report.json').read_text())
assert all(sha(ROOT / path) == digest for path, digest in latest['source_sha256'].items())
summary = {
    'audit_passed': True, 'paired_ab_source_differences': differences, 'runs': results,
    'current_runtime_source_sha256': latest['source_sha256'],
    'limits': ['audit success does not convert R0 behavior failures into passes',
               'straight low-speed subset only; no real vehicle or whole-course safety claim',
               'R0 wheel-body mismatch root cause unresolved; previous contact remains a failure'],
}
with (HERE / 'final_audit.json').open('x', encoding='utf-8') as stream:
    json.dump(summary, stream, ensure_ascii=False, indent=2)
print('5 source/stop/shutdown audits passed; 3 G1 behavior passes, 2 R0 behavior failures preserved')

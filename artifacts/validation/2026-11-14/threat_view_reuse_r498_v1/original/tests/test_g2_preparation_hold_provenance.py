"""준비 유지 변경 전후 바이트를 연결하며 이전 시험의 증거를 덮어쓰지 않는다."""
import hashlib
import json
from pathlib import Path
import zipfile

from test_source_provenance_r0_plus_b3 import m3_revision_digest


def test_preparation_hold_contract_extends_previous_source_chain():
    root = Path(__file__).resolve().parents[1]
    name = 'config/tests/g2_preparation_hold_r1_sha256.json'
    child = json.loads((root / name).read_text(encoding='utf-8'))
    sha = lambda data: hashlib.sha256(data).hexdigest()
    assert child['parent_contract'] == 'config/tests/g2_handoff_corridor_r1_sha256.json'
    assert sha((root / child['parent_contract']).read_bytes()) == child['parent_contract_sha256']
    assert sha((root / child['before_snapshot']).read_bytes()) == child['before_snapshot_sha256']
    assert set(child['approved_changes']) == {
        'src/arena_autonomy/arena_autonomy/bypass_handoff.py',
        'src/arena_autonomy/arena_autonomy/g1_g2_node.py'}
    assert not child['approved_evaluation_scripts']
    with zipfile.ZipFile(root / child['before_snapshot']) as archive:
        for path, row in child['approved_changes'].items():
            assert sha(archive.read(path)) == row['before'] != row['after']
            assert row['reason'].strip()
            assert m3_revision_digest(path, newer_than=name) == row['after']
    assert set(child['new_dependencies']) == {'src/arena_autonomy/arena_autonomy/bypass_preparation_hold.py'}
    for path, digest in child['new_dependencies'].items():
        assert m3_revision_digest(path, newer_than=name) == digest

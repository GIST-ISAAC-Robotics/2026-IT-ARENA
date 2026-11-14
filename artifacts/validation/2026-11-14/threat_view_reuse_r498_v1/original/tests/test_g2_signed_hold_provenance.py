"""기존 실패/연결 바이트를 보존하는 후속 계약. 주행 통과 주장은 별도다."""
import hashlib
import json
from pathlib import Path
import zipfile

from test_source_provenance_r0_plus_b3 import m3_revision_digest

ROOT = Path(__file__).resolve().parents[1]
NAME = 'config/tests/g2_signed_hold_r1_sha256.json'


def test_signed_hold_successor_preserves_previous_source():
    sha = lambda data: hashlib.sha256(data).hexdigest()
    child = json.loads((ROOT/NAME).read_text(encoding='utf-8'))
    assert child['parent_contract'] == 'config/tests/g2_corridor_r1_sha256.json'
    assert sha((ROOT/child['parent_contract']).read_bytes()) == child['parent_contract_sha256']
    assert sha((ROOT/child['before_snapshot']).read_bytes()) == child['before_snapshot_sha256']
    assert set(child['approved_changes']) == {'src/arena_autonomy/arena_autonomy/bypass_entry_follow.py'}
    assert not child['approved_evaluation_scripts']
    with zipfile.ZipFile(ROOT/child['before_snapshot']) as archive:
        for path, change in child['approved_changes'].items():
            assert sha(archive.read(path)) == change['before'] != change['after']
            assert m3_revision_digest(path, newer_than=NAME) == change['after']
    for path, digest in child['new_dependencies'].items():
        assert sha((ROOT/path).read_bytes()) == digest

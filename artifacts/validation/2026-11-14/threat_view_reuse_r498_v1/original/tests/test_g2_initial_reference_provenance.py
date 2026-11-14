"""초기 기준 공급자 연결 전 소스와 후속 변경의 바이트 보존."""
import hashlib
import json
from pathlib import Path
import zipfile
from test_source_provenance_r0_plus_b3 import m3_revision_digest


def test_initial_reference_contract_preserves_parent_sources():
    root=Path(__file__).resolve().parents[1];name='config/tests/g2_initial_reference_r1_sha256.json'
    child=json.loads((root/name).read_text(encoding='utf-8'));sha=lambda data:hashlib.sha256(data).hexdigest()
    assert child['parent_contract']=='config/tests/g2_preparation_hold_r1_sha256.json'
    assert sha((root/child['parent_contract']).read_bytes())==child['parent_contract_sha256']
    assert sha((root/child['before_snapshot']).read_bytes())==child['before_snapshot_sha256']
    assert len(child['approved_changes'])==5 and not child['approved_evaluation_scripts']
    with zipfile.ZipFile(root/child['before_snapshot']) as archive:
        for path,row in child['approved_changes'].items():
            assert sha(archive.read(path))==row['before']!=row['after']
            assert m3_revision_digest(path,newer_than=name)==row['after']
    assert set(child['new_dependencies'])=={'src/arena_autonomy/arena_autonomy/bypass_initial_reference.py'}
    for path,digest in child['new_dependencies'].items():assert m3_revision_digest(path,newer_than=name)==digest

def test_initial_reference_r2_preserves_failed_connection():
    root=Path(__file__).resolve().parents[1];name='config/tests/g2_initial_reference_r2_sha256.json'
    child=json.loads((root/name).read_text(encoding='utf-8'));sha=lambda data:hashlib.sha256(data).hexdigest()
    assert child['parent_contract']=='config/tests/g2_initial_reference_r1_sha256.json'
    assert sha((root/child['parent_contract']).read_bytes())==child['parent_contract_sha256']
    assert sha((root/child['before_snapshot']).read_bytes())==child['before_snapshot_sha256']
    assert set(child['approved_changes'])=={'src/arena_autonomy/arena_autonomy/bypass_entry_trial.py'}
    with zipfile.ZipFile(root/child['before_snapshot']) as archive:
        for path,row in child['approved_changes'].items():
            assert sha(archive.read(path))==row['before']!=row['after']
            assert m3_revision_digest(path,newer_than=name)==row['after']

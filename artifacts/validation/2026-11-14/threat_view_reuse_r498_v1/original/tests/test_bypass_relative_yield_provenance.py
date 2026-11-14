import hashlib
import json
from pathlib import Path
import zipfile
from test_source_provenance_r0_plus_b3 import m3_revision_digest


def test_previous_motion_pause_bytes_remain_recoverable():
    root=Path(__file__).resolve().parents[1]
    name='config/tests/g2_relative_yield_r1_sha256.json'
    c=json.loads((root/name).read_text(encoding='utf-8'))
    sha=lambda value:hashlib.sha256(value).hexdigest()
    assert sha((root/c['parent_contract']).read_bytes())==c['parent_contract_sha256']
    parent=json.loads((root/c['parent_contract']).read_text(encoding='utf-8'))
    assert sha((root/c['before_snapshot']).read_bytes())==c['before_snapshot_sha256']
    with zipfile.ZipFile(root/c['before_snapshot']) as archive:
        for path,row in c['approved_changes'].items():
            assert sha(archive.read(path))==parent['approved_changes'][path]['after']==row['before']
            assert row['before']!=row['after']==m3_revision_digest(path,newer_than=name)
    for path,digest in c['new_dependencies'].items():
        assert m3_revision_digest(path,newer_than=name)==digest


def test_motion_pause_successor_preserves_failed_original_surface_source():
    root=Path(__file__).resolve().parents[1]
    name='config/tests/g2_relative_yield_r2_sha256.json'
    c=json.loads((root/name).read_text(encoding='utf-8'))
    sha=lambda value:hashlib.sha256(value).hexdigest()
    assert sha((root/c['parent_contract']).read_bytes())==c['parent_contract_sha256']
    parent=json.loads((root/c['parent_contract']).read_text(encoding='utf-8'))
    assert sha((root/c['before_snapshot']).read_bytes())==c['before_snapshot_sha256']
    with zipfile.ZipFile(root/c['before_snapshot']) as archive:
        for path,row in c['approved_changes'].items():
            expected=parent['approved_changes'].get(path,{}).get('after',parent['new_dependencies'].get(path))
            assert sha(archive.read(path))==row['before']==expected
            assert row['before']!=row['after']==m3_revision_digest(path,newer_than=name)
    for path,digest in c['new_dependencies'].items():
        assert m3_revision_digest(path,newer_than=name)==digest

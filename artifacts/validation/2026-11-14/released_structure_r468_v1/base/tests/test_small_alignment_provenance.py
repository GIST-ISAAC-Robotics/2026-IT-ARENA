import hashlib
import json
from pathlib import Path
import zipfile
from test_source_provenance_r0_plus_b3 import m3_revision_digest


def test_selected_alignment_preserves_failure_source_and_current_bytes():
    root=Path(__file__).resolve().parents[1]
    sha=lambda b:hashlib.sha256(b).hexdigest()
    child=json.loads((root/'config/tests/small_alignment_r1_sha256.json').read_text(encoding='utf-8'))
    assert child['parent_contract']=='config/tests/actor_tail_capture_r1_sha256.json'
    assert sha((root/child['parent_contract']).read_bytes())==child['parent_contract_sha256']
    source=root/child['before_snapshot']
    assert sha(source.read_bytes())==child['before_snapshot_sha256']
    assert len(child['approved_changes'])==3
    assert child['approved_evaluation_scripts']=={} and child['new_dependencies']=={}
    with zipfile.ZipFile(source) as archive:
        for path,row in child['approved_changes'].items():
            assert row['reason'].strip()
            assert sha(archive.read(path))==row['before']!=row['after']
            assert m3_revision_digest(path, newer_than='config/tests/small_alignment_r1_sha256.json')==row['after']

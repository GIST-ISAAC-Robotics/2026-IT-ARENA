import hashlib
import json
from pathlib import Path
import zipfile
from test_source_provenance_r0_plus_b3 import m3_revision_digest


def test_tail_fix_preserves_invalid_run_source_and_verdict():
    root=Path(__file__).resolve().parents[1]
    sha=lambda data:hashlib.sha256(data).hexdigest()
    child=json.loads((root/'config/tests/actor_tail_capture_r1_sha256.json').read_text(encoding='utf-8'))
    assert child['parent_contract']=='config/tests/rotation_reobservation_r1_sha256.json'
    assert sha((root/child['parent_contract']).read_bytes())==child['parent_contract_sha256']
    snapshot=root/child['before_snapshot']
    assert sha(snapshot.read_bytes())==child['before_snapshot_sha256']
    assert child['approved_changes']=={} and child['new_dependencies']=={}
    assert set(child['approved_evaluation_scripts'])=={'scripts/validate_g1_traffic.py'}
    with zipfile.ZipFile(snapshot) as archive:
        for path,row in child['approved_evaluation_scripts'].items():
            assert sha(archive.read(path))==row['before']!=row['after']
            assert m3_revision_digest(path, newer_than='config/tests/actor_tail_capture_r1_sha256.json')==row['after']
    run=root/'artifacts/validation/2026-10-28/bump_traffic_v1'
    report=json.loads((run/'run1/report.json').read_text(encoding='utf-8'))
    audit=json.loads((run/'run1_tail_audit.json').read_text(encoding='utf-8'))
    assert not report['passed'] and not report['run_valid']
    assert sha((run/'run1/report.json').read_bytes())==audit['input_sha256']['report.json']
    # 상세 자세 원문은 로컬 제외 자료다. 사후 감사는 실행 때 불변을 확인했다.
    assert audit['input_unchanged'] and audit['tail_count_matches_original']

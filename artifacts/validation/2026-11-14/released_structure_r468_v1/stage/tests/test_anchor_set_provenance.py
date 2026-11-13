"""소스 출처 후속은 원 실행 파일과 현재 변경을 모두 대조한다."""
import hashlib
import json
from pathlib import Path
import zipfile
import pytest
from test_source_provenance_r0_plus_b3 import m3_revision_digest

ROOT=Path(__file__).resolve().parents[1]


def test_complete_anchor_successor_preserves_original_run_sources():
    digest=lambda value:hashlib.sha256(value).hexdigest()
    contract=json.loads((ROOT/'config/tests/traffic_anchor_set_r1_sha256.json').read_text())
    assert contract['parent_contract']=='config/tests/traffic_path_recovery_r0_sha256.json'
    assert digest((ROOT/contract['parent_contract']).read_bytes())==contract['parent_contract_sha256']
    archive=ROOT/contract['before_snapshot']
    assert digest(archive.read_bytes())==contract['before_snapshot_sha256']
    assert contract['approved_evaluation_scripts']=={} and contract['new_dependencies']=={}
    assert set(contract['approved_changes'])=={
        f'src/arena_autonomy/arena_autonomy/{name}.py' for name in
        ('partial_observation','traffic_recovery_trial','follow_trace')}
    with zipfile.ZipFile(archive) as before:
        for path,row in contract['approved_changes'].items():
            assert row['reason'].strip()
            assert digest(before.read(path))==row['before']
            assert m3_revision_digest(path,newer_than='config/tests/traffic_anchor_set_r1_sha256.json')==row['after']!=row['before']


@pytest.mark.parametrize('name,parent',[
    ('traffic_positive_clearance_r1_sha256.json','traffic_speed_recheck_r1_sha256.json'),
    ('traffic_positive_bundle_r1_sha256.json','traffic_positive_clearance_r1_sha256.json'),
    ('traffic_side_scope_r1_sha256.json','traffic_positive_bundle_r1_sha256.json'),
    ('traffic_manifest_r1_sha256.json','traffic_side_scope_r1_sha256.json')])
def test_positive_clearance_preserves_previous_code_and_checks_new_dependency(name,parent):
    digest=lambda value:hashlib.sha256(value).hexdigest()
    name='config/tests/'+name
    contract=json.loads((ROOT/name).read_text(encoding='utf-8'))
    assert contract['parent_contract']=='config/tests/'+parent
    assert digest((ROOT/contract['parent_contract']).read_bytes())==contract['parent_contract_sha256']
    assert digest((ROOT/contract['before_snapshot']).read_bytes())==contract['before_snapshot_sha256']
    with zipfile.ZipFile(ROOT/contract['before_snapshot']) as archive:
        for group in ('approved_changes','approved_evaluation_scripts'):
            for path,row in contract[group].items():
                assert row['reason'].strip()
                assert digest(archive.read(path))==row['before']!=row['after']
                assert m3_revision_digest(path,newer_than=name)==row['after']
    for path,value in contract['new_dependencies'].items():
        assert m3_revision_digest(path,newer_than=name)==value

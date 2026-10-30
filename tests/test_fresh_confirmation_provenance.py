"""새 명시 정책의 출처·의존성을 검증하며 과거 실패 바이트는 변경하지 않는다."""
import hashlib
import json
from pathlib import Path
import zipfile

from test_source_provenance_r0_plus_b3 import m3_revision_digest

ROOT = Path(__file__).resolve().parents[1]
NAME = 'config/tests/fresh_confirmation_r2_sha256.json'


def test_successor_and_offline_bytes():
    sha=lambda b:hashlib.sha256(b).hexdigest()
    contract=json.loads((ROOT/NAME).read_text(encoding='utf-8'))
    assert contract['parent_contract']=='config/tests/measured_lane_preference_r1_sha256.json'
    assert sha((ROOT/contract['parent_contract']).read_bytes())==contract['parent_contract_sha256']
    assert sha((ROOT/contract['before_snapshot']).read_bytes())==contract['before_snapshot_sha256']
    with zipfile.ZipFile(ROOT/contract['before_snapshot']) as archive:
        for group in ('approved_changes','approved_evaluation_scripts'):
            for path,row in contract[group].items():
                assert sha(archive.read(path))==row['before'] != row['after']
                assert m3_revision_digest(path,newer_than=NAME)==row['after']
    for path,digest in contract['new_dependencies'].items():
        assert sha((ROOT/path).read_bytes())==digest
    archive=contract['offline_origin_archive']
    assert sha((ROOT/archive['path']).read_bytes())==archive['sha256']
    with zipfile.ZipFile(ROOT/archive['path']) as stored:
        for path,digest in contract['offline_origin'].items():
            assert sha(stored.read(path))==sha((ROOT/path).read_bytes())==digest


def test_current_speed_proposal_matches_offline_source_except_label_and_preserves_parent():
    runtime=(ROOT/'src/arena_autonomy/arena_autonomy/fresh_parallel_speed_trial.py').read_text(encoding='utf-8')
    offline=(ROOT/'artifacts/validation/2026-10-30/fresh_confirmation_memory_v1/candidate_speed.py').read_text(encoding='utf-8')
    assert runtime==offline.replace('observed_measured_v1','observed_fresh_v2',1)

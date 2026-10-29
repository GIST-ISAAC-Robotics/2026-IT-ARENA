"""연속 확인 연결은 비교안과 같고, 앞선 실패 소스와 기본 OFF를 보존한다."""
import ast
import hashlib
import json
from pathlib import Path
import zipfile
from test_source_provenance_r0_plus_b3 import m3_revision_digest


ROOT = Path(__file__).resolve().parents[1]


def test_rolling_connection_preserves_failure_source_and_current_bytes():
    sha = lambda data: hashlib.sha256(data).hexdigest()
    contract = json.loads((ROOT/'config/tests/rolling_clearance_r1_sha256.json').read_text(encoding='utf-8'))
    assert contract['parent_contract'] == 'config/tests/small_alignment_r1_sha256.json'
    assert sha((ROOT/contract['parent_contract']).read_bytes()) == contract['parent_contract_sha256']
    assert sha((ROOT/contract['before_snapshot']).read_bytes()) == contract['before_snapshot_sha256']
    assert set(contract['approved_changes']) == {
        'src/arena_autonomy/arena_autonomy/anchor_clearance_trial.py',
        'src/arena_autonomy/arena_autonomy/traffic_recovery_trial.py',
    }
    assert contract['approved_evaluation_scripts'] == {} and contract['new_dependencies'] == {}
    with zipfile.ZipFile(ROOT/contract['before_snapshot']) as archive:
        for path, change in contract['approved_changes'].items():
            assert change['reason'].strip()
            assert sha(archive.read(path)) == change['before'] != change['after']
            assert m3_revision_digest(path, newer_than='config/tests/rolling_clearance_r1_sha256.json') == change['after']


def test_connected_state_machine_equals_saved_comparison_candidate():
    def class_ast(path, name):
        tree = ast.parse(path.read_text(encoding='utf-8'))
        node = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == name)
        node.name = 'ComparedStateMachine'
        return ast.dump(node, include_attributes=False)
    assert class_ast(ROOT/'src/arena_autonomy/arena_autonomy/anchor_clearance_trial.py', 'RollingClearanceTrial') == class_ast(
        ROOT/'artifacts/validation/2026-10-28/bump_traffic_v1/rolling_clearance_candidate.py', 'RollingClearanceCandidate')


def test_plain_follow_defaults_do_not_select_traffic_recovery():
    from arena_autonomy.local_follow import FollowConfig
    assert not FollowConfig().traffic_recovery_trial

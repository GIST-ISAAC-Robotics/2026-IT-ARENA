"""같은 스캔 대기 연결의 소스/이전 실패 보존·기본 OFF 계약."""
import hashlib
import json
from pathlib import Path
import zipfile
ROOT=Path(__file__).resolve().parents[1]


def test_fresh_scan_connection_preserves_failure_source_and_current_bytes():
    sha = lambda data: hashlib.sha256(data).hexdigest()
    contract = json.loads((ROOT/'config/tests/fresh_scan_wait_r1_sha256.json').read_text(encoding='utf-8'))
    assert contract['parent_contract'] == 'config/tests/rolling_clearance_r1_sha256.json'
    assert sha((ROOT/contract['parent_contract']).read_bytes()) == contract['parent_contract_sha256']
    assert sha((ROOT/contract['before_snapshot']).read_bytes()) == contract['before_snapshot_sha256']
    assert set(contract['approved_changes']) == {
        'src/arena_autonomy/arena_autonomy/traffic_recovery_trial.py',
    }
    assert contract['approved_evaluation_scripts'] == {} and contract['new_dependencies'] == {}
    with zipfile.ZipFile(ROOT/contract['before_snapshot']) as archive:
        for path, change in contract['approved_changes'].items():
            assert change['reason'].strip()
            assert sha(archive.read(path)) == change['before'] != change['after']
            assert sha((ROOT/path).read_bytes()) == change['after']



def test_default_config_keeps_recovery_unselected():
    from arena_autonomy.local_follow import FollowConfig,LocalFollow
    assert not FollowConfig().traffic_recovery_trial
    assert LocalFollow().partial_policy is None

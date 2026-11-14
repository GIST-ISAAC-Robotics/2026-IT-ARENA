import hashlib
import json

import pytest
from source_contract_helpers import hash_before_observation_extension, RECOVERY_CONTRACTS


@pytest.mark.parametrize('tamper', [None,'source','parent'])
def test_historical_hash_requires_current_source_and_parent_match(tmp_path,tamper):
    directory=tmp_path/'config/tests'
    directory.mkdir(parents=True)
    (directory/'parent.json').write_text('{}')
    (tmp_path/'source.py').write_text('current')
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    contract=dict(parent_contract='config/tests/parent.json',
                  parent_contract_sha256=digest(directory/'parent.json'),
                  newly_tracked_changes={'source.py':{'before':'old_verified_hash',
                                                     'after':digest(tmp_path/'source.py')}})
    (directory/'recent_observation_r0_sha256.json').write_text(json.dumps(contract))
    # 이 검사는 recent_observation 자체의 변조를 검사한다. 앞선 후속 단계도
    # 명시적인 빈 계약으로 준비하며, 실제 검사기의 누락 파일 거절은 유지한다.
    for name in RECOVERY_CONTRACTS:
        (directory/name).write_text(json.dumps(dict(approved_changes={}, approved_evaluation_scripts={})))
    if tamper:
        target=tmp_path/'source.py' if tamper=='source' else directory/'parent.json'
        target.write_text('unregistered change')
        with pytest.raises(AssertionError):hash_before_observation_extension(tmp_path,'source.py')
    else:
        assert hash_before_observation_extension(tmp_path,'source.py')=='old_verified_hash'
        assert hash_before_observation_extension(tmp_path,'config/tests/parent.json')==digest(directory/'parent.json')


def test_missing_newest_contract_is_not_silently_skipped(tmp_path):
    (tmp_path/'source.py').write_text('current')
    with pytest.raises(FileNotFoundError, match=RECOVERY_CONTRACTS[0]):
        hash_before_observation_extension(tmp_path,'source.py')

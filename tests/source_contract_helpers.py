"""과거 소스 계약과 명시 후속 변경의 연결. 과거 JSON/해시를 덮어쓰지 않는다."""
import hashlib
import json


def hash_before_observation_extension(root, path):
    """새 보호층 bytes까지 확인한 뒤 이전 계약 시점의 값을 반환한다.

    현재 hash를 검사하지 않고 과거 값을 돌려주는 우회가 아니다. 이 함수의 대상은
    새 관측 계약에 명시된 newly_tracked_changes뿐이며 다른 파일은 실제 hash다.
    """
    current=hashlib.sha256((root/path).read_bytes()).hexdigest()
    contract=json.loads((root/'config/tests/recent_observation_r0_sha256.json').read_text())
    change=contract['newly_tracked_changes'].get(path)
    if change is None:
        return current
    parent=root/contract['parent_contract']
    assert hashlib.sha256(parent.read_bytes()).hexdigest()==contract['parent_contract_sha256']
    assert current==change['after'], path
    return change['before']

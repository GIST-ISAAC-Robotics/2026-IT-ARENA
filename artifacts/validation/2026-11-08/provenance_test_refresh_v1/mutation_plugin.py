"""표적 변이 확인용 pytest 플러그인. 실제 파일은 바꾸지 않고 Path 읽기 결과만 가로챈다.

PROVENANCE_MUTATION:
  script_bytes  - scripts/validate_g1_traffic.py 바이트 끝에 b'\n' 추가(현재 최종 해시 불일치)
  parent_bytes  - stopped_entry_centering_r33 계약 바이트 끝에 b' ' 추가(r34의 부모 해시 불일치)
  parent_link   - g1g2_lidar_stress_r34 계약의 parent_contract를 사슬 밖 이름으로 변경(부모 연결 깨짐)
  fake_parent   - r34의 부모를 사슬의 이전 계약 moving_pass_wall_mask_r32로 바꾸고 그 실제 해시까지 맞춤(직전 부모 검사로만 잡힘)
"""
import hashlib
import json
import os
from pathlib import Path

import pytest

MODE = os.environ.get('PROVENANCE_MUTATION', '')
_read_bytes, _read_text = Path.read_bytes, Path.read_text


def _is(path, suffix):
    return Path(path).as_posix().endswith(suffix)


def read_bytes(self):
    data = _read_bytes(self)
    if MODE == 'script_bytes' and _is(self, 'scripts/validate_g1_traffic.py'):
        return data + b'\n'
    if MODE == 'parent_bytes' and _is(self, 'config/tests/stopped_entry_centering_r33_sha256.json'):
        return data + b' '
    return data


def read_text(self, *args, **kwargs):
    text = _read_text(self, *args, **kwargs)
    if MODE == 'parent_link' and _is(self, 'config/tests/g1g2_lidar_stress_r34_sha256.json'):
        row = json.loads(text)
        row['parent_contract'] = 'config/tests/not_registered_sha256.json'
        return json.dumps(row)
    if MODE == 'fake_parent' and _is(self, 'config/tests/g1g2_lidar_stress_r34_sha256.json'):
        row = json.loads(text)
        fake = 'config/tests/moving_pass_wall_mask_r32_sha256.json'
        row['parent_contract'] = fake
        row['parent_contract_sha256'] = hashlib.sha256(_read_bytes(Path(fake))).hexdigest()
        return json.dumps(row)
    return text


@pytest.fixture(autouse=True)
def _mutate(monkeypatch):
    assert MODE in ('script_bytes', 'parent_bytes', 'parent_link', 'fake_parent')
    monkeypatch.setattr(Path, 'read_bytes', read_bytes)
    monkeypatch.setattr(Path, 'read_text', read_text)

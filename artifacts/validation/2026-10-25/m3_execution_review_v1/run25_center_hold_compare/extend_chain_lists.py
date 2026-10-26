"""출처 사슬 목록 끝에 새 후속 계약 이름만 덧붙인다. 바이트(CRLF 포함)를 보존하고 각 파일의 치환 횟수를 확인한다."""
from pathlib import Path
OLD = b"'m3_release_gate_r0_sha256.json')"
NEW = b"'m3_release_gate_r0_sha256.json', 'm3_small_center_r0_sha256.json')"
EXPECTED = {'tests/test_corner_follow.py': 1, 'tests/test_g1_dense_stop.py': 1, 'tests/test_physics_step_option.py': 2}
for name, count in EXPECTED.items():
    p = Path(name)
    data = p.read_bytes()
    assert data.count(OLD) == count, (name, data.count(OLD))
    p.write_bytes(data.replace(OLD, NEW))
    print(name, 'replaced', count)
p = Path('tests/test_source_provenance_r0_plus_b3.py')
data = p.read_bytes()
old = b"'config/tests/m3_release_gate_r0_sha256.json')"
assert data.count(old) == 1
p.write_bytes(data.replace(old, b"'config/tests/m3_release_gate_r0_sha256.json',\n                 'config/tests/m3_small_center_r0_sha256.json')"))
print(p, 'M3_SUCCESSORS extended')

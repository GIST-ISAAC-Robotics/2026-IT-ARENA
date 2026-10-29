"""현재 바이트 검증을 유지하면서 기존 출처 검사에 새 계약을 연결한다."""
from pathlib import Path
ROOT = Path(__file__).resolve().parents[4]
for name in ('test_source_provenance_r0_plus_b3.py', 'test_corner_follow.py',
             'test_g1_dense_stop.py', 'test_physics_step_option.py'):
    path = ROOT/'tests'/name
    text = path.read_text(encoding='utf-8')
    prefix = 'config/tests/' if name == 'test_source_provenance_r0_plus_b3.py' else ''
    old = "'"+prefix+"traffic_recovery_trial_r0_sha256.json')"
    new = "'"+prefix+"traffic_recovery_trial_r0_sha256.json', '"+prefix+"traffic_recovery_release_fringe_r1_sha256.json')"
    expected = 2 if name == 'test_physics_step_option.py' else 1
    assert text.count(old) == expected, (name, text.count(old))
    path.write_text(text.replace(old, new), encoding='utf-8')

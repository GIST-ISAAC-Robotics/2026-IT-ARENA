"""직전 r41의 실행·평가 조건을 보존하고 새 증거 경로를 준비한다."""
from pathlib import Path

A=Path(__file__).resolve().parent;OLD=A.parent/'current_corner_support_v1'
for name in ('run.py','check_runtime.py','assess.py','new_track_course.json','evaluation_source.json','geometry.json'):
    assert not (A/name).exists()
    (A/name).write_bytes((OLD/name).read_bytes())
s=(OLD/'run.sh').read_text().replace('A=artifacts/validation/2026-11-08/current_corner_support_v1',
    'A=artifacts/validation/2026-11-08/corner_occlusion_connected_r45_v1')
(A/'run.sh').write_text(s,newline='\n')
s=(OLD/'register.py').read_text().replace('current_corner_support_r41_sha256.json','corner_occlusion_r45_sha256.json')
s=s.replace("legacy = A.parent/'fragment_boundary_connected_v1/right_parallel_first_curve/registered.json'",
    "legacy = A.parent/'current_corner_support_v1/registered.json'")
s=s.replace("A.parent/'fragment_boundary_connected_v1/new_track_course.json'","A.parent/'current_corner_support_v1/new_track_course.json'")
s=s.replace("'tests/test_corner_boundary_support.py',", "'tests/test_corner_occlusion_support.py', 'tests/test_corner_boundary_support.py',")
s=s.replace("print('r41 sources'", "print('r45 sources'")
s=s.replace("scope='r41 current two-face support of valid retained corner; r30 same32s side actor'",
    "scope='r45 current foreground witness extends valid two-face corner; r41 same32s side actor'")
needle="contract = 'config/tests/corner_occlusion_r45_sha256.json'"
s=s.replace(needle,"""assert (A/'tests_v1.exit').read_text().strip() == '0'
assert (A/'replay_v1.exit').read_text().strip() == '0'
assert all(v['mismatch']==0 for v in json.loads((A/'replay_summary.json').read_bytes()).values())
"""+needle)
(A/'register.py').write_text(s,newline='\n')
print('r41 same-condition execution prepared; not registered or run')

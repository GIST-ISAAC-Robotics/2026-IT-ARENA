"""r200 종료/소스 불변 확인 뒤 검증한 입력 변환만 적용한다."""
import hashlib
import json
from pathlib import Path
import zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
integrity=json.loads((A.parent/'released_gap_runtime_r200_v1/two_actor_run1/extra_integrity.json').read_bytes())
assert integrity==dict(changed=[],snapshot_unchanged=True)
manifest=json.loads((A/'candidate_sources.json').read_bytes())
source=ROOT/manifest['source'];candidate=A/'candidate_c1_native_preview.py'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(source)==manifest['before_sha256'] and sha(candidate)==manifest['candidate_sha256']
test=ROOT/'tests/test_c1_native_preview.py'
with zipfile.ZipFile(A/'before_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in (source,test):z.write(p,p.relative_to(ROOT).as_posix())
source.write_bytes(candidate.read_bytes())
text=test.read_text('utf8')
old="assert scan.ranges_m[0]==1 and math.isnan(scan.ranges_m[1]) and math.isnan(scan.ranges_m[2])"
new="assert scan.ranges_m[0]==1 and scan.ranges_m[1]==math.inf and math.isnan(scan.ranges_m[2])"
assert text.count(old)==1
text=text.replace(old,new)
start=(A/'test_candidate.py').read_text('utf8')
start=start[start.index('@pytest.mark.parametrize'):start.index('def test_all_saved_raw_values_change_only_confirmed_zero_distance')]
start=start.replace('candidate.C1NativePreview','C1NativePreview')
text=text.replace('from arena_autonomy.native_lidar_input import decode_measurements',
                  'from arena_autonomy.native_lidar_input import decode_measurements\nfrom arena_autonomy.local_scene import scene_from_measurements')
test.write_text(text+'\n\n'+start,encoding='utf8')
with (A/'applied_sources.json').open('x') as f:
    json.dump(dict(before_snapshot_sha256=sha(A/'before_sources.zip'),after={str(p.relative_to(ROOT).as_posix()):sha(p) for p in (source,test)},
        candidate_byte_identical=sha(source)==sha(candidate)),f,indent=2)
print('Applied raw-backed nonreturn encoding; source matches tested candidate.')

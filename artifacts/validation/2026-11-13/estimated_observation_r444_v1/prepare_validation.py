"""Windows 읽기 전용 소스 대조와 r444 전용 검증 묶음. 공통 파일은 수정하지 않는다."""
from pathlib import Path
import ast
import difflib
import hashlib
import json
import sys
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
VERSION=sys.argv[1] if len(sys.argv)>1 else 'v1'
assert VERSION in ('v1','v2','v3')
def sha(data):return hashlib.sha256(data).hexdigest()
before=json.loads((BASE/'source_before.json').read_bytes())
changes={};diff=[]
for rel,want in before.items():
    data=(ROOT/rel).read_bytes();assert sha(data)==want,rel
    original=BASE/'original'/rel;original.parent.mkdir(parents=True,exist_ok=True)
    if original.exists():assert original.read_bytes()==data
    else:original.write_bytes(data)
    updated=(BASE/'stage'/rel).read_bytes();ast.parse(updated.decode('utf-8'))
    changes[rel]=dict(original_sha256=want,stage_sha256=sha(updated))
    diff.extend(difflib.unified_diff(data.decode('utf-8').splitlines(True),updated.decode('utf-8').splitlines(True),
                                   fromfile='a/'+rel,tofile='b/'+rel))
for file in (BASE/'stage/tests').glob('*.py'):
    rel=file.relative_to(BASE/'stage').as_posix();data=file.read_bytes();ast.parse(data.decode())
    if rel in changes:continue
    assert not (ROOT/rel).exists(),rel
    changes[rel]=dict(original_sha256=None,stage_sha256=sha(data))
    diff.extend(difflib.unified_diff([],data.decode().splitlines(True),fromfile='/dev/null',tofile='b/'+rel))
(BASE/'apply.patch').write_text(''.join(diff),encoding='utf-8',newline='\n')
(BASE/'changes.json').write_text(json.dumps(dict(staged_only=True,files=changes),indent=2)+'\n')
files={}
for package in ('arena_autonomy','arena_vehicle_interface'):
    for p in (ROOT/'src'/package/package).rglob('*.py'):files[p.relative_to(ROOT).as_posix()]=p.read_bytes()
for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('_numeric_kernels*.so'):files[p.relative_to(ROOT).as_posix()]=p.read_bytes()
assert any(name.endswith('.so') for name in files)
for name in ('test_current_corner_runtime.py','test_estimated_motion_time.py','test_native_measurement_motion.py',
             'test_lidar_motion_ros.py','test_current_path_recheck.py'):
    files['tests/'+name]=(ROOT/'tests'/name).read_bytes()
for p in (BASE/'stage').rglob('*.py'):files[p.relative_to(BASE/'stage').as_posix()]=p.read_bytes()
fixture='artifacts/validation/2026-11-12/released_runtime_r168_v1/two_actor_run1/run1/follow_decisions.jsonl'
files[fixture]=(ROOT/fixture).read_bytes()
for p in (BASE.parent/'crop_masked_recovery_r438_v1/inputs').iterdir():files['inputs/'+p.name]=p.read_bytes()
files['inputs/r440_rows.jsonl']=(BASE.parent/'crop_current_crawl_r440_v1/verify_v1/saved_result/rows.jsonl').read_bytes()
for name in ('verify_saved.py','run_validation.py'):
    files[name]=(BASE/name).read_bytes();ast.parse(files[name].decode('utf-8'))
manifest={name:sha(data) for name,data in files.items()}
archive=BASE/f'validation_sources_{VERSION}.zip'
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
    for name,data in files.items():z.writestr(name,data)
    z.writestr('manifest.json',json.dumps(manifest,indent=2))
(BASE/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(BASE/f'manifest_{VERSION}.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(dict(files=len(files),changes=len(changes),archive_sha256=sha(archive.read_bytes()))))

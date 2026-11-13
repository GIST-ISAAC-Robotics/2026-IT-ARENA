"""v1의 동결 입력을 그대로 보존하고 geometry 한 모듈만 v2로 교체. WSL 실행 없음."""
import ast,copy,hashlib,json,zipfile
from pathlib import Path
root=Path(__file__).resolve().parents[1];dest=root/'execution_v2'
dest.mkdir(exist_ok=False)
sha=lambda b:hashlib.sha256(b).hexdigest()
before=json.loads((root/'execution_v1/prepared.json').read_bytes())
assert sha((root/'execution_v1/source.zip').read_bytes())==before['source_archive_sha256']
with zipfile.ZipFile(root/'execution_v1/source.zip') as z:inputs={name:z.read(name) for name in z.namelist()}
rel='src/arena_autonomy/arena_autonomy/vehicle_marker_geometry.py'
inputs[rel]=(root/'stage'/rel).read_bytes();ast.parse(inputs[rel])
after=copy.deepcopy(before)
after.update(base_archive_sha256=before['source_archive_sha256'],source_delta=[rel],
    revision='v2_duplicate_black_code_evidence',execution_authorized=False,
    execution_waits_for_root_WSL_allocation=True)
for row in after['files']:
    row['sha256']=sha(inputs[row['path']])
    if row['path']==rel:row['basis']='r466_v2_geometry_only'
with zipfile.ZipFile(dest/'source.zip','w',zipfile.ZIP_DEFLATED) as z:
    for name,data in sorted(inputs.items()):z.writestr(name,data)
after['source_archive_sha256']=sha((dest/'source.zip').read_bytes())
after['planned_shell']=after['planned_shell'].replace('execution_v1','execution_v2')
(dest/'prepared.json').write_text(json.dumps(after,ensure_ascii=False,indent=2)+'\n')
script=(root/'execution_v1/run_once.sh').read_text().replace('execution_v1','execution_v2').replace('it-arena-r466-ros-a','it-arena-r466-ros-b')
(dest/'run_once.sh').write_text(script,encoding='utf-8',newline='\n')
print(json.dumps(dict(source_archive_sha256=after['source_archive_sha256'],
    prepared_sha256=sha((dest/'prepared.json').read_bytes()),source_delta=after['source_delta']),indent=2))

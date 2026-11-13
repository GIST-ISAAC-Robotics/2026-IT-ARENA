"""Windows 읽기 원본 + r466 stage를 독립 ROS 검증 입력 ZIP으로 고정. WSL 실행 없음."""
import ast
import hashlib
import json
from pathlib import Path
import zipfile

here=Path(__file__).resolve().parent;artifact=here.parent;repo=artifact.parents[3]
receipt=artifact/'execution_v1';receipt.mkdir(exist_ok=True)
sha=lambda b:hashlib.sha256(b).hexdigest()
inputs={};rows=[]
for package in ('arena_autonomy','arena_vehicle_interface'):
    for source in sorted((repo/'src'/package/package).rglob('*.py')):
        rel=source.relative_to(repo).as_posix();staged=artifact/'stage'/rel
        selected=staged if staged.exists() else source
        data=selected.read_bytes();ast.parse(data,filename=rel)
        inputs[rel]=data;rows.append(dict(path=rel,basis='r466_stage' if staged.exists() else 'read_only_common',
            source_path=str(selected),sha256=sha(data)))
for source in sorted((artifact/'stage/src/arena_autonomy/arena_autonomy').glob('vehicle_marker*.py')):
    rel=source.relative_to(artifact/'stage').as_posix()
    if rel in inputs:continue
    data=source.read_bytes();ast.parse(data,filename=rel);inputs[rel]=data
    rows.append(dict(path=rel,basis='r466_stage_new',source_path=str(source),sha256=sha(data)))
for source,rel in [(here/'run_ros_validation.py','validation/run_ros_validation.py'),
        (repo/'tests/test_local_follow.py','tests/test_local_follow.py'),
        (artifact/'stage/config/perception/vehicle_marker_approx_b0390.json','config/vehicle_marker_approx_b0390.json')]:
    data=source.read_bytes();inputs[rel]=data
    if source.suffix=='.py':ast.parse(data,filename=rel)
    rows.append(dict(path=rel,basis='test_input',source_path=str(source),sha256=sha(data)))
for row in rows:
    if sha(Path(row['source_path']).read_bytes())!=row['sha256']:
        raise RuntimeError('source changed during snapshot: '+row['path'])
with zipfile.ZipFile(receipt/'source.zip','w',zipfile.ZIP_DEFLATED) as z:
    for rel,data in sorted(inputs.items()):z.writestr(rel,data)
manifest=dict(scope='isolated synthetic ROS representative, no build/no drive hardware',files=rows,
    source_archive_sha256=sha((receipt/'source.zip').read_bytes()),
    tests=['two previously excluded ROS-import cases','absent','front','side','TTL expiry','final zero'],
    platform='Ubuntu-24.04 / ROS Jazzy',domain=96,localhost_only=True,execution_authorized=False,
    execution_waits_for_root_WSL_allocation=True,build_required=False,
    planned_shell="wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/vehicle_marker_r466_v1/execution_v1/run_once.sh'")
(receipt/'prepared.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(files=len(rows),source_archive_sha256=manifest['source_archive_sha256'],
    execution_authorized=False,build_required=False),indent=2))

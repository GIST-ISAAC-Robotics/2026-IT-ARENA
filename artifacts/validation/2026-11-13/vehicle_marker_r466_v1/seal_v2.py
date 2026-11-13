import ast,difflib,hashlib,json,zipfile
from pathlib import Path
root=Path(__file__).resolve().parent;rev=root/'revision_v2';repo=root.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
changes=[];diff=[];delta=[];delta_files=[]
for p in sorted((root/'stage').rglob('*')):
    if not p.is_file() or p.suffix not in ('.py','.json'):continue
    rel=p.relative_to(root/'stage').as_posix();old=root/'original'/rel;v1=rev/'stage_before'/rel
    if p.suffix=='.py':ast.parse(p.read_text(encoding='utf-8'),filename=rel)
    if not old.exists() or sha(old)!=sha(p):
        changes.append(dict(path=rel,original_sha256=sha(old) if old.exists() else None,stage_sha256=sha(p)))
        diff.extend(difflib.unified_diff(old.read_text(encoding='utf-8').splitlines(True) if old.exists() else [],
            p.read_text(encoding='utf-8').splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
    if not v1.exists() or sha(v1)!=sha(p):
        delta_files.append(dict(path=rel,before_sha256=sha(v1) if v1.exists() else None,after_sha256=sha(p)))
        delta.extend(difflib.unified_diff(v1.read_text(encoding='utf-8').splitlines(True) if v1.exists() else [],
            p.read_text(encoding='utf-8').splitlines(True),fromfile='v1/'+rel,tofile='v2/'+rel))
(rev/'changes.json').write_text(json.dumps(changes,indent=2)+'\n')
(rev/'delta_from_v1.json').write_text(json.dumps(delta_files,indent=2)+'\n')
(rev/'stage.diff').write_text(''.join(diff),encoding='utf-8')
(rev/'delta_from_v1.diff').write_text(''.join(delta),encoding='utf-8')
with zipfile.ZipFile(rev/'stage_source.zip','w',zipfile.ZIP_DEFLATED) as z:
    for row in changes:z.write(root/'stage'/row['path'],row['path'])
preserved=json.loads((rev/'preserved.json').read_bytes())
assert all(sha(root/name)==digest for name,digest in preserved.items())
prepared=json.loads((root/'execution_v2/prepared.json').read_bytes())
assert prepared['files'][next(i for i,r in enumerate(prepared['files']) if r['path'].endswith('/vehicle_marker_geometry.py'))]['sha256']==sha(root/'stage/src/arena_autonomy/arena_autonomy/vehicle_marker_geometry.py')
result=dict(status='v2_windows_verified_waiting_for_WSL_assignment',windows_tests=32,windows_exit_code=0,
    common_modified=False,ros_v2_executed=False,device_accessed=False,v1_preserved=True,
    integration_files=len(changes),delta_files=delta_files,stage_archive_sha256=sha(rev/'stage_source.zip'),
    ros_source_archive_sha256=prepared['source_archive_sha256'],ros_prepared_sha256=sha(root/'execution_v2/prepared.json'),
    pending='same bounded ROS representative after root resource allocation',
    calibration_short_range='one frontal sample at .2 or .5m accepted; synthetic contract, no measured accuracy')
(rev/'result.json').write_text(json.dumps(result,indent=2)+'\n')
(root/'latest_revision.json').write_text(json.dumps(dict(revision='revision_v2',result='revision_v2/result.json',
    changes='revision_v2/changes.json',archive='revision_v2/stage_source.zip',ros_execution='execution_v2',status=result['status']),indent=2)+'\n')
print(json.dumps(result,indent=2))

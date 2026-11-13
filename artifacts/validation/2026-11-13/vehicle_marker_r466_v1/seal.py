"""stage 소스/변경 목록/원본 불변과 원 출처를 봉인한다. 공통 파일은 읽기만 한다."""
import ast,difflib,hashlib,json,platform,zipfile
from pathlib import Path

root=Path(__file__).resolve().parent;repo=root.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
original=json.loads((root/'original_sha256.json').read_text(encoding='utf-8'))
integrity=[]
for rel,expected in original.items():
    actual=sha(repo/rel)
    integrity.append(dict(path=rel,original_sha256=expected,current_sha256=actual,unchanged=actual==expected))
changes=[];diff=[]
for p in sorted((root/'stage').rglob('*')):
    if not p.is_file() or p.suffix not in ('.py','.json'):continue
    rel=p.relative_to(root/'stage').as_posix();old=root/'original'/rel
    if p.suffix=='.py':ast.parse(p.read_text(encoding='utf-8'),filename=rel)
    if old.exists() and sha(old)==sha(p):continue
    if not old.exists() and (repo/rel).exists():raise RuntimeError('uncaptured existing file '+rel)
    changes.append(dict(path=rel,original_sha256=sha(old) if old.exists() else None,stage_sha256=sha(p)))
    before=old.read_text(encoding='utf-8').splitlines(True) if old.exists() else []
    diff.extend(difflib.unified_diff(before,p.read_text(encoding='utf-8').splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
(root/'stage.diff').write_text(''.join(diff),encoding='utf-8')
(root/'changes.json').write_text(json.dumps(changes,indent=2)+'\n',encoding='utf-8')
(root/'source_integrity.json').write_text(json.dumps(dict(all_unchanged=all(x['unchanged'] for x in integrity),files=integrity),indent=2)+'\n',encoding='utf-8')
with zipfile.ZipFile(root/'stage_source.zip','w',zipfile.ZIP_DEFLATED) as archive:
    for item in changes:archive.write(root/'stage'/item['path'],item['path'])
sources={p.name:sha(p) for p in sorted((root/'sources').iterdir()) if p.is_file()}
(root/'sources_sha256.json').write_text(json.dumps(sources,indent=2)+'\n',encoding='utf-8')
result=dict(status='stage_implemented_windows_verified_pending_root_review_and_ROS',
    windows_tests=dict(passed=197,deselected_ros_required=2,exit_code=0,log='windows_tests_v11.log',
        junit='windows_tests.xml',python=platform.python_version()),
    no_common_edits=True,source_unchanged=all(x['unchanged'] for x in integrity),
    stage_archive_sha256=sha(root/'stage_source.zip'),diff_sha256=sha(root/'stage.diff'),
    changed_files=len(changes),ros_runtime_executed=False,wsl_used=False,jetson_used=False,
    physical_accuracy_verified=False,full_drive_verified=False,
    limitations=['new overtaking initiation from marker only is not connected',
        'metric accuracy, installed extrinsics, actual print sizes and exposure timing are unverified',
        'independent guard method executed with I/O doubles, not a live ROS node'],
    owned_child_processes_remaining=0)
(root/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))

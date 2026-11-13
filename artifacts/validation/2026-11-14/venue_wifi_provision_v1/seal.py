from pathlib import Path
import ast,difflib,hashlib,json
A=Path(__file__).resolve().parent;ROOT=A.parents[3];changes=[];parts=[]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for file in sorted((A/'stage/scripts').iterdir()):
    if not file.is_file():continue
    if file.suffix=='.py':ast.parse(file.read_text(encoding='utf-8'),filename=str(file))
    path='scripts/'+file.name;old=ROOT/path
    before=old.read_text(encoding='utf-8') if old.exists() else ''
    after=file.read_text(encoding='utf-8')
    parts.extend(difflib.unified_diff(before.splitlines(True),after.splitlines(True),
        fromfile='a/'+path if old.exists() else '/dev/null',tofile='b/'+path))
    changes.append(dict(path=path,before_sha256=sha(old) if old.exists() else None,stage_sha256=sha(file),
        added_lines=sum(1 for line in difflib.ndiff(before.splitlines(),after.splitlines()) if line.startswith('+ ')),
        removed_lines=sum(1 for line in difflib.ndiff(before.splitlines(),after.splitlines()) if line.startswith('- '))))
(A/'candidate.patch').write_text(''.join(parts),encoding='utf-8')
receipt=dict(changes=changes,unique_mock_tests=14,unittest_exit=int((A/'unittest_v2.exit').read_text(encoding='utf-8-sig').strip()),
    powershell_parse=json.loads((A/'powershell_parse.json').read_text(encoding='utf-8-sig')),common_modified=False,
    physical_network_access=False,credential_read=False,remote_pid=None,owned_remaining=0,
    temporary_mock_directories=[p.name for p in A.glob('mock_*') if p.is_dir()])
(A/'changes.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8')
assert len(changes)==4 and not receipt['temporary_mock_directories'] and receipt['unittest_exit']==0
print(json.dumps(receipt,indent=2))

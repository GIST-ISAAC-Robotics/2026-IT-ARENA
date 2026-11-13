"""보존 base/staged의 문법·현재 원문을 확인하고 검토용 patch/hash를 갱신한다."""
import ast
import difflib
import hashlib
import json
from pathlib import Path

A=Path(__file__).resolve().parent
ROOT=A.parents[3]


def digest(raw):return hashlib.sha256(raw).hexdigest()


def main():
    files={};patch=[]
    for path in sorted((A/'staged').rglob('*.py')):
        name=path.relative_to(A/'staged').as_posix();base=A/'base'/name
        raw=path.read_bytes();ast.parse(raw,filename=name)
        original=base.read_bytes() if base.exists() else b''
        if base.exists():assert (ROOT/name).read_bytes()==original,('common changed during preparation',name)
        else:assert not (ROOT/name).exists(),('new module now exists',name)
        before=original.decode('utf8').replace('\r\n','\n');after=raw.decode('utf8').replace('\r\n','\n')
        patch.extend(difflib.unified_diff(before.splitlines(True),after.splitlines(True),
            fromfile='a/'+name if original else '/dev/null',tofile='b/'+name))
        files[name]=dict(base_sha256=digest(original) if original else None,staged_sha256=digest(raw))
    assert len(files)==9
    (A/'geometry_contract.patch').write_text(''.join(patch),encoding='utf8',newline='\n')
    (A/'stage_manifest.json').write_text(json.dumps(dict(files=files,common_sources_modified=False,
        default_hardware_profile_changed=False,vehicle_yaml_modified=False,controller_calls=0),indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(staged_files=len(files),patch_sha256=digest((A/'geometry_contract.patch').read_bytes()))))


if __name__=='__main__':main()

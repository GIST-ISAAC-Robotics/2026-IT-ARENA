"""대상 코드를 실행하지 않는 AST 의존 묶음 준비. 동결 인계 후 다시 실행하지 않는다."""
import ast
import difflib
import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def digest(data): return hashlib.sha256(data).hexdigest()


def main():
    assert (ROOT/'AGENTS.md').is_file()
    assert not (HERE/'bundle_sources.zip').exists(), 'prepare once; never overwrite a frozen bundle'
    spec = importlib.util.spec_from_file_location('r403_static_dependency_closure',ROOT/'scripts/python_dependency_closure.py')
    closure_module = importlib.util.module_from_spec(spec); spec.loader.exec_module(closure_module)
    staged = {p.relative_to(HERE/'staged').as_posix():p for p in (HERE/'staged').rglob('*.py')
              if p.name != '__init__.py'}
    assert len(staged) == 7
    overlays = {**staged, **{'tests/'+p.name:p for p in (HERE/'tests').glob('*.py')}}
    class Overlay:
        def __contains__(self,name): return name in overlays or (ROOT/name).is_file()
        def __getitem__(self,name): return overlays.get(name,ROOT/name).read_bytes()
    selection = json.loads((HERE/'test_selection.json').read_bytes())
    entries = sorted({name.split('::')[0] for name in selection})
    closure = closure_module.collect(entries,roots=closure_module.PACKAGE_ROOTS+closure_module.SCRIPT_ROOTS+closure_module.TEST_ROOTS,
                                     members=Overlay())
    members = {name:overlays.get(name,ROOT/name).read_bytes() for name in closure['files']}
    members['verify_tests.py'] = (HERE/'verify_tests.py').read_bytes()
    for name,data in members.items():
        if name.endswith('.py'): ast.parse(data,filename=name)
    provenance = {name:dict(origin=(overlays.get(name,ROOT/name).relative_to(ROOT).as_posix()
                                   if name!='verify_tests.py' else (HERE/'verify_tests.py').relative_to(ROOT).as_posix()),
                           sha256=digest(data),staged=name in overlays)
                  for name,data in members.items()}
    original = {}; patches = []
    for name,path in sorted(staged.items()):
        before = (ROOT/name).read_bytes() if (ROOT/name).is_file() else b''
        if before: original[name] = before
        patches.extend(difflib.unified_diff(before.decode('utf8').replace('\r\n','\n').splitlines(True),path.read_text(encoding='utf8').splitlines(True),
            fromfile='a/'+name,tofile='b/'+name))
    frozen = dict(schema='r403_runtime_frame_bundle_v1',files={name:digest(data) for name,data in members.items()},
        test_nodeids=selection,staged_modules=sorted(Path(name).stem for name in staged),
        original_sources={name:digest(data) for name,data in original.items()},
        package_init='original common __init__; staged extend_path is only an optional overlay shim',
        source_provenance=provenance,closure=closure,executed=False)
    (HERE/'source_changes.patch').write_text(''.join(patches),encoding='utf8',newline='\n')
    (HERE/'bundle_manifest.json').write_text(json.dumps(frozen,ensure_ascii=False,indent=2)+'\n',encoding='utf8',newline='\n')
    with zipfile.ZipFile(HERE/'before_sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name,data in original.items(): archive.writestr(name,data)
    with zipfile.ZipFile(HERE/'bundle_sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name,data in sorted(members.items()): archive.writestr(name,data)
        archive.write(HERE/'bundle_manifest.json','bundle_manifest.json')
    print(json.dumps(dict(source_files=len(members),staged_modules=len(staged),test_files=len(entries),
        bundle_sha256=digest((HERE/'bundle_sources.zip').read_bytes()),algorithm_executed=False)))


if __name__ == '__main__': main()

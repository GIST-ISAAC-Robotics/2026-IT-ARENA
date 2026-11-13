"""대상 정책을 실행하지 않는 원문/AST/입력 묶음 준비. 인계 뒤 실행하지 않는다."""
import ast
import difflib
import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]


def sha(data):return hashlib.sha256(data).hexdigest()


def main():
    assert not (HERE/'bundle_sources.zip').exists()
    spec=importlib.util.spec_from_file_location('r405_static_closure',ROOT/'scripts/python_dependency_closure.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    staged={p.relative_to(HERE/'staged').as_posix():p for p in (HERE/'staged').rglob('*.py')}
    assert len(staged)==8
    overlays={**staged,**{'tests/'+p.name:p for p in (HERE/'tests').glob('*.py')}}
    class Source:
        def __contains__(self,name):return name in overlays or (ROOT/name).is_file()
        def __getitem__(self,name):return overlays.get(name,ROOT/name).read_bytes()
    entries=list(staged)+['tests/test_current_corner_runtime.py','tests/test_runtime_node_backend.py']
    closure=module.collect(entries,roots=module.PACKAGE_ROOTS+module.SCRIPT_ROOTS+module.TEST_ROOTS,members=Source())
    members={name:overlays.get(name,ROOT/name).read_bytes() for name in closure['files']}
    for name in ('verify_tests.py','compare_current.py'):members[name]=(HERE/name).read_bytes()
    members.update({p.relative_to(HERE).as_posix():p.read_bytes() for p in (HERE/'inputs').glob('*') if p.is_file()})
    for name,data in members.items():
        if name.endswith('.py'):ast.parse(data,filename=name)
    for name in module.NATIVE_MODULES['arena_autonomy._numeric_kernels']:
        assert name in members
    source_ledger={}
    original={};diff=[]
    for name,path in sorted(staged.items()):
        before=(ROOT/name).read_bytes() if (ROOT/name).is_file() else b''
        if before:original[name]=before
        source_ledger[name]=dict(staged_sha256=sha(path.read_bytes()),common_before_sha256=sha(before) if before else None)
        diff.extend(difflib.unified_diff(before.decode('utf8').replace('\r\n','\n').splitlines(True),path.read_text(encoding='utf8').splitlines(True),fromfile='a/'+name,tofile='b/'+name))
    reference={}
    frozen=ROOT/'artifacts/validation/2026-11-13/current_corner_budget_r386_v1/revision_final_geometry_v1/bundle_sources.zip'
    with zipfile.ZipFile(frozen) as z:
        for name in ('candidate.py','budget.py'):reference['r386/'+name]=z.read('r386/'+name)
    package=HERE/'staged/src/arena_autonomy/arena_autonomy/race_runtime_policy'
    assert (package/'corner_budget.py').read_bytes()==reference['r386/budget.py']
    candidate=reference['r386/candidate.py'].decode('utf8').replace('\r\n','\n')
    candidate=candidate.replace('from arena_autonomy.current_path_recheck._gaps import bounded_body_nonreturn\n','')
    candidate=candidate.replace('from budget import propose,elapsed','from .corner_budget import propose,elapsed')
    candidate=candidate.replace('*,core_rebase,','*,core_rebase,proof_callback,')
    candidate=candidate.replace('proof=bounded_body_nonreturn(','proof=proof_callback(').replace("measured,core['speed_mps'],relative_neighbor=True)","measured,core['speed_mps'])")
    candidate=candidate.replace('저장 비교 전용.','명시 선택형 현재 계산.')
    assert candidate==(package/'corner_candidate.py').read_text(encoding='utf8')
    component_root=ROOT/'artifacts/validation/2026-11-13/range_mask_component_r396_v1/full_current_v3'
    for name in ('range_mask_component.py','profile_evidence.py'):
        reference['r396/'+name]=(component_root/'staged'/name).read_bytes()
        expected=reference['r396/'+name].decode('utf8').replace('\r\n','\n').replace('from profile_evidence import','from .profile_evidence import')
        assert expected==(package/name).read_text(encoding='utf8')
    reference['r396/execution_source_audit.md']=(component_root/'verify_v1/execution_source_audit/REVIEW.md').read_bytes()
    source_ledger['reference_functions']={name:sha(data) for name,data in reference.items()}
    source_ledger['changes_allowed']='relative imports, explicit proof callback replacing ignored relative_neighbor keyword, selection-only docstring'
    (HERE/'source_ledger.json').write_text(json.dumps(source_ledger,ensure_ascii=False,indent=2)+'\n',encoding='utf8',newline='\n')
    (HERE/'source_changes.patch').write_text(''.join(diff),encoding='utf8',newline='\n')
    bundle=dict(schema='r405_current_corner_bundle_v1',files={name:sha(data) for name,data in members.items()},
        closure=closure,staged_files=list(staged),input_provenance=json.loads((HERE/'input_provenance.json').read_bytes()),
        source_ledger=source_ledger,executed=False)
    (HERE/'bundle_manifest.json').write_text(json.dumps(bundle,ensure_ascii=False,indent=2)+'\n',encoding='utf8',newline='\n')
    with zipfile.ZipFile(HERE/'before_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name,data in original.items():z.writestr(name,data)
    with zipfile.ZipFile(HERE/'reference_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name,data in reference.items():z.writestr(name,data)
    with zipfile.ZipFile(HERE/'bundle_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name,data in sorted(members.items()):z.writestr(name,data)
        z.write(HERE/'bundle_manifest.json','bundle_manifest.json')
    print(json.dumps(dict(frozen_files=len(members),staged_files=len(staged),bundle_sha256=sha((HERE/'bundle_sources.zip').read_bytes()),algorithm_executed=False)))


if __name__=='__main__':main()

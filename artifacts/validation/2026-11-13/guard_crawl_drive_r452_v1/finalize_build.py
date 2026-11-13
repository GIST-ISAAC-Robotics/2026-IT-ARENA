"""성공 build/import와 실제 delta를 연결하고 root 동결 검토용 receipt 초안을 만든다."""
from pathlib import Path
import importlib.util,json
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
spec=importlib.util.spec_from_file_location('r452_prepare',A/'prepare.py');p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
b=A/'build_v1'
assert (b/'build.exit').read_text().strip()==(b/'import.exit').read_text().strip()=='0'
imports=p.read(b/'import_paths.log');assert imports['native_import_verified'] and imports['preview_default_off']
files={path.relative_to(ROOT).as_posix():p.sha(path) for path in sorted(b.iterdir()) if path.is_file()}
wsl_root='/mnt/'+ROOT.drive[0].lower()+ROOT.as_posix()[2:]
native=imports['native_module'];assert native['path'].startswith(wsl_root+'/')
name=native['path'][len(wsl_root)+1:];assert p.sha(ROOT/name)==native['sha256'];files[name]=native['sha256']
receipt=dict(build_executed=True,build_exitcode=0,import_exitcode=0,native_import_verified=True,
    packages=['arena_autonomy','arena_vehicle_interface','arena_bringup'],files=files,
    source_scope_sha256=p.sha(A/'source_scope.json'),import_paths=imports,
    self_underlay_override_warning_preserved=True,simulation_started=False)
p.write_new(b/'receipt.json',receipt)
actual=p.draft_review();p.write_new(A/'source_review_actual_v1.json',actual)
scope=p.read(A/'source_scope.json')
template=dict(actual,root_freeze_approved=False)
template['changes']=[dict(row,reason=scope['files'][row['path']]['reason']) for row in actual['changes']]
p.write_new(A/'root_source_review_template.json',template)
print(json.dumps(dict(build_receipt=True,delta_count=len(actual['changes']),source_scope_sha256=p.sha(A/'source_scope.json'),
    root_review_template='root_source_review_template.json',simulation_started=False)))

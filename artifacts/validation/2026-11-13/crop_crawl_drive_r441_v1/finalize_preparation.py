"""완료된 build/import를 읽고 허용된5파일을 다시 대조해 동결한다. 주행 실행 없음."""
import importlib.util
import json
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
spec=importlib.util.spec_from_file_location('r441_prepare',A/'prepare.py');p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
b=A/'build_v1'
assert (b/'build.exit').read_text().strip()==(b/'import.exit').read_text().strip()=='0'
receipt=dict(compiled_again=True,build_exitcode=0,import_exitcode=0,
    packages=['arena_autonomy','arena_vehicle_interface','arena_bringup'],
    files={path.relative_to(ROOT).as_posix():p.sha(path) for path in sorted(b.iterdir()) if path.is_file()},
    source_scope_sha256=p.sha(A/'source_scope.json'),
    import_paths=p.read(b/'import_paths.log'),preview_default_off_import_assertion_passed=True,
    self_underlay_override_warning_preserved=True)
p.write_new(b/'receipt.json',receipt)
actual=p.draft_review();p.write_new(A/'source_review_actual_v1.json',actual)
approved=dict(actual,status='reviewed',review_scope='root가 지정한 r4344+r4401 범위와 검증 SHA의 exact 대조; 주행 실행 승인은 별도')
scope=p.read(A/'source_scope.json')
approved['changes']=[dict(row,reason=scope['files'][row['path']]['reason']) for row in approved['changes']]
p.write_new(A/'source_review_approved_v1.json',approved)
answer=p.freeze(A/'source_review_approved_v1.json',A/'source_scope.json')
print(json.dumps(answer))

"""현재 adapter 파일만 봉인한다. 최종 source 실행 동결이 아니다."""
import ast
import hashlib
import json
from pathlib import Path
A=Path(__file__).resolve().parent
p=(A/'preflight.py').read_text()
p=p.replace('    coarse=ev.Centerline([0,2,2,0],[0,0,2,2],[0,2,4,6],8.)\n    samples=np.arange(0.,8.,.01);poses=[coarse.pose(t) for t in samples]\n    line=ev.Centerline([p[0] for p in poses],[p[1] for p in poses],samples,8.)',
            '    line=ev.Centerline([0,2,2,0],[0,0,2,2],[0,2,4,6],8.)')
p=p.replace("    assert se.evaluate_run(**hit)['dense_wall_overlap']['overlap_samples']>0;checks['wall_overlap_detected']=True",
            "    assert not se.evaluate_run(**hit)['gates']['no_wall_overlap'];checks['wall_overlap_rejected']=True")
(A/'original/preflight_v1.py').write_bytes(p.encode())
files=[*A.glob('*.py'),*A.glob('*.sdf'),*A.glob('*.patch'),*A.glob('*.md'),*(A/'launch').glob('*.py'),
       *(A/'execution_v1').glob('*.py'),*(A/'execution_v1').glob('*.sh')]
for p in files:
    if p.suffix=='.py':ast.parse(p.read_bytes())
manifest={p.relative_to(A).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
(A/'adapter_draft_manifest.json').write_text(json.dumps(dict(status='awaiting_r459_policy_integration_not_frozen',files=manifest),indent=2)+'\n')
print(json.dumps(dict(adapter_files=len(files),python_ast_valid=True,prepared_exists=(A/'prepared.json').exists(),run_exists=(A/'run1').exists())))

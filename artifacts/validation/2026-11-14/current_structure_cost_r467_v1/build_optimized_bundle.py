"""후속 승인: r463 stage 대 helper 최적화, 신규 r467-b에서 24+2회만 계산."""
import ast
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

A=Path(__file__).resolve().parent
O=A/'optimization_v1'
B=O/'bundle_v2'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert json.loads((O/'local_verification.json').read_bytes())['passed']
B.mkdir(exist_ok=False)
for version,source in (('base',A/'bundle/stage'),('stage',O/'stage')):
    shutil.copytree(source,B/version)
shutil.copyfile(A/'bundle/inputs.json',B/'inputs.json')
shutil.copyfile(A/'bundle/environment_reference.py',B/'environment_reference.py')
original_results=json.loads((A/'remote_result/results/stage.json').read_bytes())
expected={r['id']:r['output'] for r in original_results['rows'] if r['round']==0}
(B/'expected_arm_outputs.json').write_text(json.dumps(expected))
runner=(A/'run_cost.py').read_text(encoding='utf8')
runner=runner.replace('import os\n','import os\nimport resource\n')
runner=runner.replace("    records, headers = data['records'], data['headers']", "    records, headers = data['records'], data['headers']\n    expected = json.loads((ROOT/'expected_arm_outputs.json').read_bytes())")
runner=runner.replace("        for round_index in range(3):", "        assert warmup_output == expected[first['id']]\n        for round_index in range(3):")
runner=runner.replace("                assert digest(requests[key]) == hashes[key]\n                assert output", "                assert digest(requests[key]) == hashes[key]\n                assert output == expected[key], (key, round_index, 'original ARM output changed')\n                assert output")
runner=runner.replace('        measured_calls=12, warmup_calls=1, instrumentation_output_exact=True,',
    '        measured_calls=12, warmup_calls=1, instrumentation_output_exact=True,\n        original_arm_output_exact=True, process_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,')
(B/'run_cost.py').write_text(runner,encoding='utf8',newline='\n')
executor=(A/'execute_package.py').read_text(encoding='utf8').replace("Path('/home/q/it-arena-r467-a')","Path('/home/q/it-arena-r467-b')")
(B/'execute_package.py').write_text(executor,encoding='utf8',newline='\n')
files={p.relative_to(B).as_posix():sha(p) for p in sorted(B.rglob('*')) if p.is_file()}
baseline={p[len('base/'):]:h for p,h in files.items() if p.startswith('base/')}
optimized={p[len('stage/'):]:h for p,h in files.items() if p.startswith('stage/')}
assert baseline.keys()==optimized.keys()
changes=[dict(path=p,before=baseline[p],after=optimized[p]) for p in baseline if baseline[p]!=optimized[p]]
assert changes==json.loads((O/'changes.json').read_bytes())
native=json.loads((A/'source_manifest.json').read_bytes())['native_reuse']
for p,h in native['original_source_sha256'].items():assert baseline[p]==optimized[p]==h
manifest=dict(files=files,policy_changes=changes,native_reuse=native,input_ids=list(expected),
    computations=24,warmups=2,rounds=3,root_reviewed=True,jetson_exclusive_resource_assigned=True,
    previous_r467a_results_sha256=sha(A/'remote_result/results/stage.json'),
    local_verification_sha256=sha(O/'local_verification.json'),
    physical_measurement=False,comparison='r463 stage versus helper optimization only',
    instrumentation='round0 original functions; rounds1/2 pass-through wrappers; all outputs exact to previous ARM')
for p in B.rglob('*.py'):ast.parse(p.read_bytes())
for p in (B/'source_manifest.json',O/'source_manifest.json'):p.write_text(json.dumps(manifest,indent=2))
archive=O/'bundle_sources.zip'
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(B.rglob('*')):
        if p.is_file():z.write(p,p.relative_to(B).as_posix())
result=dict(archive_sha256=sha(archive),manifest_sha256=sha(O/'source_manifest.json'),
    input_sha256=sha(B/'inputs.json'),source_count=len(files),changes=changes,computations=24,warmups=2)
(O/'prepared.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result))

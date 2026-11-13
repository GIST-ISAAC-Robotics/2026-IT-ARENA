"""r467 최적화와 r470 native 후보를 한 번의 동일 ARM 비교로 고정한다."""
import ast,hashlib,json
from pathlib import Path
import shutil,zipfile

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
R467=A.parent/'current_structure_cost_r467_v1'
B=A/'bundle'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
B.mkdir(exist_ok=False)
for name,source in (('base',R467/'optimization_v1/stage'),('stage',A/'stage')):
    shutil.copytree(source,B/name)
for name in ('inputs.json','expected_arm_outputs.json','environment_reference.py'):
    shutil.copyfile(R467/'optimization_v1/bundle_v2'/name,B/name)
runner=(R467/'optimization_v1/bundle_v2/run_cost.py').read_text(encoding='utf8')
runner=runner.replace('from arena_autonomy import pass_release_trial as ps, known_structure',
    'from arena_autonomy import pass_release_trial as ps, known_structure, numeric_kernels')
runner=runner.replace("                    install(known_structure, 'current_mask_shadow')",
    "                    install(known_structure, 'current_mask_shadow')\n                if hasattr(numeric_kernels, 'current_mask_occupied'):\n                    install(numeric_kernels, 'current_mask_occupied')")
(B/'run_cost.py').write_text(runner,encoding='utf8',newline='\n')
for name in ('execute_package.py','verify_native.py'):shutil.copyfile(A/name,B/name)
shutil.copyfile(ROOT/'tests/test_current_structure_shadow.py',B/'test_current_structure_shadow_reference.py')
assert sha(B/'test_current_structure_shadow_reference.py')=='e742acd67a6f8f4ff5b4b174d30bb824d6ef2e9501a55c96908d7a2174f48172'
files={p.relative_to(B).as_posix():sha(p) for p in sorted(B.rglob('*')) if p.is_file()}
changes=json.loads((A/'changes.json').read_bytes())
actual=[]
for path in sorted((B/'stage').rglob('*')):
    if path.is_file():
        rel=path.relative_to(B/'stage');before=B/'base'/rel
        old=sha(before) if before.exists() else None
        if old!=sha(path):actual.append(dict(path=rel.as_posix(),before=old,after=sha(path)))
assert actual==changes
native=json.loads((R467/'source_manifest.json').read_bytes())['native_reuse']
manifest=dict(files=files,changes=changes,native_reuse=native,
    root_reviewed=True,jetson_exclusive_resource_assigned=True,computations=24,warmups=2,
    native_helper_verification=True,expected_arm_output_source_sha256=sha(R467/'optimization_v1/remote_result/results/stage.json'),
    input_sha256=sha(B/'inputs.json'),new_packages=False,new_native_build=True,
    comparison='r467 optimized vs same source with actual-hit padding predicate native only')
for path in B.rglob('*.py'):ast.parse(path.read_bytes())
for path in (B/'source_manifest.json',A/'source_manifest.json'):path.write_text(json.dumps(manifest,indent=2))
archive=A/'bundle_sources.zip'
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(B.rglob('*')):
        if p.is_file():z.write(p,p.relative_to(B).as_posix())
report=dict(archive_sha256=sha(archive),manifest_sha256=sha(A/'source_manifest.json'),
    input_sha256=sha(B/'inputs.json'),files=len(files),policy_changes=0,numeric_predicate_changes=1,
    total_evaluator_calls=24,warmups=2)
(A/'prepared.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))

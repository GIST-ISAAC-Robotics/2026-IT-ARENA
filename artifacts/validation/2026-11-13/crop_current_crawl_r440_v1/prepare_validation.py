from pathlib import Path
import ast
import hashlib
import json
import zipfile

ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3]
R438=ROOT.parent/'crop_masked_recovery_r438_v1';R426=ROOT.parent/'crop_current_reach_r426_v1'
TARGET='src/arena_autonomy/arena_autonomy/race_runtime_policy/crop_current_candidate.py'
EXPECTED='886f4875a11cc5f7cc0af4924a99717a323464542ffb4fa48e45f218fe01d9e5'
def sha(data):return hashlib.sha256(data).hexdigest()
assert sha((REPO/TARGET).read_bytes())==EXPECTED
files={}
for package in ('arena_autonomy','arena_vehicle_interface'):
    for path in (REPO/'src'/package/package).rglob('*.py'):files[path.relative_to(REPO).as_posix()]=path.read_bytes()
for path in (REPO/'src/arena_autonomy/arena_autonomy').glob('_numeric_kernels*.so'):files[path.relative_to(REPO).as_posix()]=path.read_bytes()
for path in (R438/'inputs').iterdir():files['inputs/'+path.name]=path.read_bytes()
records=[json.loads(line) for line in (R438/'inputs/r437_currents.jsonl').open(encoding='utf-8')]
files['inputs/r437_representative.json']=json.dumps(next(row for row in records if row['trace_seq']==643)).encode()
prior=[json.loads(line) for line in (R438/'result_v1/rows.jsonl').open(encoding='utf-8')]
files['inputs/r438_expected.json']=json.dumps({row['dataset']+':'+str(row['trace_seq']):dict(
    eligible=row['eligible'],fixed_01_crop=row['trials'].get('0.1',{}).get('result',{}).get('crop')) for row in prior}).encode()
with zipfile.ZipFile(R426/'bundle_sources.zip') as z:
    for name in ('tests/test_crop_current_reach.py','tests/reference_r423_scope.py','inputs/header.json','inputs/representative.json'):
        files['legacy/'+name]=z.read(name)
for path in [ROOT/'verify_saved.py',ROOT/'run_validation.py',*list((ROOT/'tests').glob('*.py'))]:
    ast.parse(path.read_text(encoding='utf-8'));files[path.relative_to(ROOT).as_posix()]=path.read_bytes()
manifest={name:sha(data) for name,data in files.items()}
with zipfile.ZipFile(ROOT/'validation_sources_v1.zip','x',zipfile.ZIP_DEFLATED) as z:
    for name,data in files.items():z.writestr(name,data)
    z.writestr('manifest.json',json.dumps(manifest,indent=2))
(ROOT/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps(dict(files=len(files),target_sha256=EXPECTED,source_sha256=sha((ROOT/'validation_sources_v1.zip').read_bytes()))))

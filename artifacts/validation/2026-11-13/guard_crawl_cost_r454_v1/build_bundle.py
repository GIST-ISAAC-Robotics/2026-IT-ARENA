"""최신 공통 사본과 원 104행/기대출력만 동결한다. 제어 계산 없음."""
from pathlib import Path
import ast,hashlib,json,shutil,zipfile
BASE=Path(__file__).resolve().parent; REPO=BASE.parents[3]; DAY=BASE.parent
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    bundle=BASE/'bundle';bundle.mkdir(exist_ok=False);files={};provenance={}
    def put(source,name):
        p=bundle/name;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,p)
        files[name]=digest(p);provenance[name]=dict(source=source.relative_to(REPO).as_posix(),sha256=digest(source))
    for pkg in ('arena_autonomy','arena_vehicle_interface'):
        for p in sorted((REPO/'src'/pkg/pkg).rglob('*')):
            if p.is_file() and p.suffix in ('.py','.cpp','.hpp','.h'):put(p,p.relative_to(REPO).as_posix())
    put(REPO/'src/arena_autonomy/setup.py','src/arena_autonomy/setup.py')
    for name in ('run_cost.py','remote_bootstrap.py'):put(BASE/name,name)
    put(DAY/'estimated_observation_cost_r447_v1/remote_bootstrap.py','r447_bootstrap_reference.py')
    put(DAY/'guard_local_cap_r451_v1/analyze.py','r451_analyze_reference.py')
    put(DAY/'guard_local_cap_r451_v1/result_integrated_v2/checks.json','inputs/expected.json')
    trace=DAY/'guard_crawl_saved_r445_v1/stage_v3/inputs/follow_decisions.jsonl'
    selected=[]; header=None
    with trace.open(encoding='utf8') as stream:
        for line in stream:
            row=json.loads(line)
            if header is None:header=row['data']
            if row['kind']=='call' and 895<=row['seq']<=998:selected.append(line.rstrip('\r\n'))
    assert len(selected)==104
    (bundle/'inputs/selected.jsonl').write_text('\n'.join(selected)+'\n',encoding='utf8')
    (bundle/'inputs/header.json').write_text(json.dumps(header)+'\n',encoding='utf8')
    for name in ('inputs/selected.jsonl','inputs/header.json'):files[name]=digest(bundle/name)
    prior=json.loads((DAY/'estimated_observation_cost_r447_v1/source_manifest.json').read_bytes())
    native={n:h for n,h in files.items() if '/_native/' in n or n=='src/arena_autonomy/setup.py'}
    assert native and all(prior['files'][n]==h for n,h in native.items())
    for p in bundle.rglob('*.py'):ast.parse(p.read_text(encoding='utf8'))
    manifest=dict(files=files,provenance=provenance,native_sources_match_r447=native,
        original_trace_sha256=digest(trace),selected_rows=104,expected_source='r451 result_integrated_v2',
        planned_original_guard_calls=312,planned_additional_guard_calls=104,warmup_calls=0)
    (bundle/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
    shutil.copyfile(bundle/'source_manifest.json',BASE/'source_manifest.json')
    archive=BASE/'bundle_sources.zip'
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(bundle.rglob('*')):
            if p.is_file():z.write(p,p.relative_to(bundle).as_posix())
    receipt=dict(archive_sha256=digest(archive),manifest_sha256=digest(BASE/'source_manifest.json'),files=len(files),rows=104,
        native_inputs_unchanged=len(native),archive_bytes=archive.stat().st_size)
    (BASE/'prepared.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf8');print(json.dumps(receipt))
if __name__=='__main__':main()

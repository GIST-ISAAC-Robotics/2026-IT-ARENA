from pathlib import Path
import hashlib,json,shutil,zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3];B=A/'bundle';B.mkdir()
prior=ROOT/'artifacts/validation/2026-11-14/jetson_plan_cost_r482_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();files={};provenance={}
def put(p,name):
    d=B/name;d.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,d)
    files[name]=sha(d);provenance[name]=dict(path=str(p.relative_to(ROOT)),sha256=sha(p))
for pkg in ('arena_autonomy','arena_vehicle_interface'):
    for p in (ROOT/'src'/pkg/pkg).rglob('*'):
        if p.is_file() and p.suffix in ('.py','.cpp','.hpp','.h'):put(p,'base/'+p.relative_to(ROOT).as_posix())
put(ROOT/'src/arena_autonomy/setup.py','base/src/arena_autonomy/setup.py')
for name in ('state_codec.py','environment_reference.py'):put(prior/name,name)
put(A/'run_profile.py','run_profile.py')
d=json.loads((prior/'profile_inputs.json').read_bytes());seqs={220,229,232,235}
selected=[x for x in d['selected'] if x['key'][1] in seqs]
for r in selected:
    name=f"{r['key'][0]}_{r['key'][1]}.pickle"
    put(prior/'profile_result_v2/states_v2'/name,'states/'+name)
r482=json.loads((prior/'compare_result/compare_results/stage_0.json').read_bytes())
data=dict(header=d['header'],selected=selected,expected_r482=[{k:r[k] for k in ('key','output')} for r in r482['rows'] if r['key'][1] in seqs])
(B/'inputs.json').write_text(json.dumps(data,allow_nan=False)+'\n',encoding='utf-8');files['inputs.json']=sha(B/'inputs.json')
reuse=json.loads((prior/'profile_manifest_v2.json').read_bytes())['native_reuse']
for p,h in reuse['source_sha256'].items():assert sha(B/'base'/p)==h,p
s=(prior/'execute_profile_v2.py').read_text(encoding='utf-8').replace('it-arena-r482-a','it-arena-r488-a')
s=s.replace('profile_manifest_v2.json','manifest.json').replace('profile_v2','profile').replace('profile-v2','profile')
s=s.replace('profile_results_v2','profile_results').replace("*(A/'states_v2').glob('*')","A/'manifest.json'")
s=s.replace("assert dest.exists() and sha(dest)==reuse['sha256']","assert not dest.exists();shutil.copy2(native,dest);assert sha(dest)==reuse['sha256']")
(A/'execute_profile.py').write_text(s,encoding='utf-8');put(A/'execute_profile.py','execute_profile.py')
m=dict(files=files,provenance=provenance,native_reuse=reuse,profile_calls=4,wall_calls=4)
(B/'manifest.json').write_text(json.dumps(m,indent=2)+'\n',encoding='utf-8');shutil.copy2(B/'manifest.json',A/'manifest.json')
with zipfile.ZipFile(A/'bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in B.rglob('*'):
        if p.is_file():z.write(p,p.relative_to(B).as_posix())
print(json.dumps(dict(files=len(files),sha256=sha(A/'bundle.zip'))))

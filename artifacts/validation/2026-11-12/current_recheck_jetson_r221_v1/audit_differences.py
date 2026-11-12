"""원 Jetson4개 불일치만 상세 재계산한다. 원 보고서는 덮어쓰지 않는다."""
from pathlib import Path
import hashlib,json,sys
BASE=Path(__file__).resolve().parent;ROOT=BASE/'worktree'
A=ROOT/'artifacts/validation/2026-11-12/current_recheck_jetson_r221_v1'
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call,freeze,equal_output
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy import guard_observation_policy as policy
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((BASE/'bundle_manifest.json').read_bytes())
unchanged=lambda:all(sha(ROOT/name)==row['sha256'] for name,row in manifest['files'].items())
assert unchanged()
report=json.loads((A/'report.json').read_bytes());selected=set(report['pc_full_mismatches'])
rows=[json.loads(line) for line in (A/'samples.jsonl').open()]
owner,motion=make_controller(rows[0]['header'])
def differences(a,b,path=''):
    if equal_output(a,b):return []
    if isinstance(a,dict) and isinstance(b,dict):
        return [r for k in sorted(set(a)|set(b)) for r in differences(a.get(k),b.get(k),path+'/'+k)]
    if isinstance(a,list) and isinstance(b,list) and len(a)==len(b):
        return [r for i,(x,y) in enumerate(zip(a,b)) for r in differences(x,y,path+'/'+str(i))]
    return [dict(path=path,actual=a,expected=b)]
results=[]
for row in rows[1:]:
    if row['seq'] not in selected:continue
    data,scan,history=restore_call(row['current'],motion)
    scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
        epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
    actual=freeze(policy.rebase(row['original']['output'],row['original']['input'],row['current'],scene,history,owner.config))
    results.append(dict(seq=row['seq'],differences=differences(actual,row['expected']),actual=actual,expected=row['expected']))
result=dict(original_failed_sequences=sorted(selected),rows=results,source_unchanged=unchanged(),
    input_sha256=sha(A/'samples.jsonl'),scope='targeted same-source replay; original full-output failure preserved')
out=BASE/'differences.json'
with out.open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
print(json.dumps(dict(rows=[dict(seq=r['seq'],differences=r['differences']) for r in results],
    source_unchanged=result['source_unchanged'],sha256=sha(out))))
assert result['source_unchanged'] and len(results)==len(selected)

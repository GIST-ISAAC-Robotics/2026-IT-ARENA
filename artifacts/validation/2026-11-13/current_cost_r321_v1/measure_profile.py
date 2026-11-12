"""r296 고정 대표106개 현재 재검사 프로파일. 계측 비용은 실시간 성능 수치가 아니다."""
from pathlib import Path
import cProfile,hashlib,importlib.util,json,pstats,sys
A=Path(__file__).resolve().parent; R=A.parents[3]
B=A.parent/'current_input_jetson_r296_v1'
spec=importlib.util.spec_from_file_location('prior_cost_measure',B/'measure_cost.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
old.configure('native_v1')
manifest=json.loads((B/'source_manifest.json').read_bytes())
assert all(hashlib.sha256((R/p).read_bytes()).hexdigest()==h for p,h in manifest.items())
headers=json.loads((B/'headers.json').read_bytes())
configs={k:(old.MotionConfig(**v['motion_config']),old.FollowConfig(**v['follow_config'])) for k,v in headers.items()}
entries=json.loads((B/'selected_inputs.json').read_bytes())
profile=cProfile.Profile();equal=0
def evaluate(entry):
    c=entry['input'];motion,cfg=configs[entry['source']]
    current=c['current_rebase']['input'];d,scan,history=old.restore_call(current,motion)
    scene=old.motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    return old.freeze(old.candidate.rebase(c['output'],old.plan_reference(c['input']),current,scene,history,cfg,require_current_region=True))
for entry in entries:
    out=profile.runcall(evaluate,entry)
    assert old.equal_output(out,entry['output']),(entry['source'],entry['index'])
    equal+=1
profile.dump_stats(str(A/'current.prof'))
with (A/'profile.txt').open('w') as f:pstats.Stats(profile,stream=f).strip_dirs().sort_stats('cumtime').print_stats(45)
summary=dict(inputs=len(entries),equal=equal,scope='instrumented fixed input; no realtime latency claim',
             selected_sha256=hashlib.sha256((B/'selected_inputs.json').read_bytes()).hexdigest())
(A/'result.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary))

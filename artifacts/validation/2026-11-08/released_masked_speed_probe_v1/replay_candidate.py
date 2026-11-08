"""부모 출력 전체 재현 + 새 사본은 최초 변경까지. 저장입력 이후 진행은 주장하지 않는다."""
from pathlib import Path
import hashlib,importlib.util,json,sys,time
from arena_autonomy.follow_trace import MotionConfig,FollowConfig,replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.pass_release_trial import PassReleaseSelector
A=Path(__file__).resolve().parent
def load(name,file):
    spec=importlib.util.spec_from_file_location(name,A/file);m=importlib.util.module_from_spec(spec)
    sys.modules[name]=m;spec.loader.exec_module(m);return m
helper=load('arena_autonomy.masked_motion_speed','masked_motion_speed.py')
candidate=load('candidate_pass_release','pass_release_candidate.py')
original=PassReleaseSelector._select
traces={
 'r37_B':A.parent/'c1_query_side_scope_v1/B_c1_injection/run1/follow_decisions.jsonl',
 'r34_A':A.parent/'g1g2_lidar_stress_pair_v1/A_no_injection/run1/follow_decisions.jsonl',
}
result={}
for label,trace in traces.items():
    old_count=new_count=0;first=None;started=time.perf_counter()
    for line in trace.open():
        e=json.loads(line);d=e['data']
        if e['kind']=='header':
            motion=MotionConfig(**d['motion_config']);cfg=FollowConfig(**d['follow_config'])
            old,new=[BypassHandoffFollow(cfg,HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True) for _ in range(2)]
        elif e['kind']=='reset':old.reset();new.reset()
        elif e['kind']=='call':
            PassReleaseSelector._select=original
            actual=freeze(replay_call(old,d['input'],motion));old_count+=1
            assert equal_output(comparable_decision(actual),comparable_decision(d['output'])),('baseline mismatch',label,d['input']['reference_s'])
            if first is not None:continue
            PassReleaseSelector._select=candidate.PassReleaseSelector._select
            changed=freeze(replay_call(new,d['input'],motion));new_count+=1
            if not equal_output(comparable_decision(changed),comparable_decision(d['output'])):
                first=dict(t=d['input']['reference_s'],old=d['output'],new=changed,scope='first divergence, not a driven result')
    result[label]=dict(original_matched=old_count,candidate_calls=new_count,first_difference=first,wall_s=time.perf_counter()-started,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),candidate_sha256=hashlib.sha256((A/'pass_release_candidate.py').read_bytes()).hexdigest())
    (A/'replay_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(label,'matched',old_count,'first',first and first['t'],flush=True)
PassReleaseSelector._select=original
assert result['r37_B']['first_difference'] is not None
assert result['r34_A']['first_difference'] is None

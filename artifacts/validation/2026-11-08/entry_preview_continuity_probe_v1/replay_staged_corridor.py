"""짧은 최초 전이 사본: r36을 재현하고 실제 출력 명령의 첫 차이까지만 비교."""
from pathlib import Path
import copy,hashlib,importlib.util,json,sys,time,types,zipfile
from arena_autonomy.follow_trace import MotionConfig,FollowConfig,replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.bypass_corridor_trial import CorridorGuide
from arena_autonomy.bypass_entry_trial import straight_reference
from arena_autonomy.pass_release_trial import PassReleaseSelector
from short_transition_candidate import compare_short_transition

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
def preserved_module(archive,member,contract_key,name):
    c=json.loads((ROOT/contract_key).read_bytes())
    with zipfile.ZipFile(A.parent/archive/'before.zip') as z:source=z.read(member)
    assert hashlib.sha256(source).hexdigest()==c['approved_changes'][member]['before']
    m=types.ModuleType(name);sys.modules[name]=m;exec(compile(source,name,'exec'),m.__dict__);return m
old_scope=preserved_module('c1_query_side_scope_v1','src/arena_autonomy/arena_autonomy/same_path_gap_trial.py',
    'config/tests/query_local_side_scope_r37_sha256.json','arena_autonomy.same_path_gap_trial')
old_select=preserved_module('masked_motion_speed_v1','src/arena_autonomy/arena_autonomy/pass_release_trial.py',
    'config/tests/masked_motion_speed_r38_sha256.json','preserved_r37_pass')
PassReleaseSelector._select=old_select.PassReleaseSelector._select
original_plan=CorridorGuide._plan
helper_spec=importlib.util.spec_from_file_location('arena_autonomy.short_entry_transition',A/'short_transition_candidate.py')
helper=importlib.util.module_from_spec(helper_spec);sys.modules[helper_spec.name]=helper;helper_spec.loader.exec_module(helper)
spec=importlib.util.spec_from_file_location('corridor_candidate',A/'bypass_corridor_candidate.py')
candidate=importlib.util.module_from_spec(spec);sys.modules[spec.name]=candidate;spec.loader.exec_module(candidate)
candidate_plan=candidate.CorridorGuide._plan
trace=A.parent/'entry_preview_wiring_v1/run1/follow_decisions.jsonl'
matched=0;new_count=0;first_detail=None;first_command=None;start=time.perf_counter()
for line in trace.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        mc=MotionConfig(**d['motion_config']);cfg=FollowConfig(**d['follow_config'])
        old,new=[BypassHandoffFollow(cfg,HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True) for _ in range(2)]
    elif e['kind']=='reset':old.reset();new.reset()
    elif e['kind']=='call':
        CorridorGuide._plan=original_plan
        actual=freeze(replay_call(old,d['input'],mc));matched+=1
        assert equal_output(comparable_decision(actual),comparable_decision(d['output'])),('old mismatch',d['input']['reference_s'])
        if first_command is not None:break
        CorridorGuide._plan=candidate_plan
        changed=freeze(replay_call(new,d['input'],mc));new_count+=1
        if first_detail is None and not equal_output(comparable_decision(changed),comparable_decision(actual)):
            first_detail=dict(t=d['input']['reference_s'],old=actual,new=changed)
        if not equal_output(changed[:2],actual[:2]):first_command=dict(t=d['input']['reference_s'],old=actual,new=changed)
CorridorGuide._plan=original_plan
result=dict(scope='exact r36 parent, candidate only until first changed drive command',matched=matched,candidate_calls=new_count,
    first_detail_difference=first_detail,first_command_difference=first_command,wall_s=time.perf_counter()-start,
    trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),helper_sha256=hashlib.sha256((A/'short_transition_candidate.py').read_bytes()).hexdigest())
(A/'staged_corridor_replay.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print('matched',matched,'detail',first_detail and first_detail['t'],'command',first_command and first_command['t'],flush=True)

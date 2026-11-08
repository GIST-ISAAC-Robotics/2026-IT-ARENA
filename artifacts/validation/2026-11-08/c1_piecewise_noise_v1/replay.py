"""완료 입력 재생: 무주입 A 전체 불변, 주입 B는 최초 차이까지만 분석."""
from pathlib import Path
import importlib.util, json, sys, time
from arena_autonomy.follow_trace import MotionConfig, FollowConfig, replay_call, freeze, comparable_decision, equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig

A=Path(__file__).resolve().parent
name='arena_autonomy.piecewise_wall'
spec=importlib.util.spec_from_file_location(name,A/'piecewise_wall.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);sys.modules[name]=module
results={}
for label in ('A_no_injection','B_c1_injection'):
    trace=A.parent/'g1g2_lidar_stress_pair_v1'/label/'run1/follow_decisions.jsonl'
    count=0;first=None;start=time.perf_counter()
    for line in trace.open():
        e=json.loads(line);d=e['data']
        if e['kind']=='header':
            motion=MotionConfig(**d['motion_config'])
            engine=BypassHandoffFollow(FollowConfig(**d['follow_config']),HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
        elif e['kind']=='reset':engine.reset()
        elif e['kind']=='call':
            actual=freeze(replay_call(engine,d['input'],motion));count+=1
            if not equal_output(comparable_decision(actual),comparable_decision(d['output'])):
                first=dict(t=d['input']['reference_s'],old=d['output'],new=actual)
                break
    results[label]=dict(calls=count,first_difference=first,wall_s=time.perf_counter()-start,
        scope='saved inputs; no continuation beyond first changed decision')
    print(label,count,'first_difference',None if first is None else first['t'],flush=True)
    (A/'replay_result.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
assert results['A_no_injection']['first_difference'] is None
assert results['B_c1_injection']['first_difference'] is not None

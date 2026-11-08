"""실패 기록의 정상 재생과 같은 센서·이력에서의 격리 진입 검사. 실제 차량에는 연결하지 않는다."""
import copy,hashlib,json,collections
from pathlib import Path
from arena_autonomy.follow_trace import MotionConfig,FollowConfig,replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
A=Path(__file__).resolve().parent; trace=A.parent/'ops_traffic_resume_v1/run1/follow_decisions.jsonl'
report=json.loads((trace.parent/'report.json').read_bytes());resume=next(s['truth'][0] for s in report['steps'] if s['step']=='resume_closed')
stats=collections.Counter();steers=[]; probes=[]; P=(13.02,14.02,16.02,19.02)
def summary(out):
    speed,steer,d=out; b=d.get('bypass') or {};ad=d.get('entry_admission') or {}
    return dict(speed=speed,steering=steer,mode=d.get('handoff_state'),reason=d.get('reason'),bypass=b.get('reason'),selected=b.get('selected'),admission=ad.get('reason'),admitted=ad.get('admitted'),bypass_keys=list(b),candidate_reasons=[(k,b[k]) for k in ('entry_phase','entry_plan_reason','path_source','wall_reason') if k in b])
for n,l in enumerate(trace.open()):
    e=json.loads(l);assert e['seq']==n;d=e['data']
    if e['kind']=='header':
        cfg=HandoffExecutionConfig.from_mapping(d['execution_config']);m=MotionConfig(**d['motion_config'])
        f=BypassHandoffFollow(FollowConfig(**d['follow_config']),cfg,allow_simulation_trial=True)
    elif e['kind']=='reset':f.reset()
    elif e['kind']=='call':
        i=d['input'];t=i['reference_s'];expected=d['output']
        if t>resume and expected[0]==0 and expected[2].get('persistence',{}).get('ready') and expected[2].get('follow_entry_space',{}).get('reason')=='observed_entry_distance_reserve':
            stats['stationary_patch_plus_distance_hold_after_resume']+=1
            steers.append(i['previous_steering_rad'])
            stats['above_start_angle_and_no_bypass']+=(abs(steers[-1])>.08 and 'bypass' not in expected[2])
        if t<20.:
            if any(abs(t-q)<1e-6 for q in P):
                o={'t':t,'speed_input':i['speed_mps'],'original':summary(expected),'previous_rad':i['previous_steering_rad']}
                for label,prev,lim in [('same_angle_test_downstream',i['previous_steering_rad'],.45),('aligned_hypothesis',0.,.08)]:
                    isolated=copy.deepcopy(f)
                    if lim!=.08:
                        # Bypass only the initial admission comparison on a detached replay instance.
                        # This does NOT change the controller, commanded steer, max servo rate, clearance, or defaults.
                        object.__setattr__(isolated.execution,'maximum_initial_steering_rad',lim)
                    o[label]=summary(replay_call(isolated,{**i,'previous_steering_rad':prev},m))
                probes.append(o)
            actual=freeze(replay_call(f,i,m)); stats['replayed_before_20s']+=1
            stats['different']+=not equal_output(comparable_decision(actual),comparable_decision(expected))
        stats['all_calls']+=1
    elif e['kind']=='footer':
        assert d['failed']==d['dropped']==d['capture_errors']==d['pending']==0 and not d['worker_alive'] and d['written']==n
assert len(probes)==len(P) and not stats['different']
data=dict(scope='장시간 고착 계수와 20초 전까지 원출력 재생. 가설은 동일 입력 4시점의 한 호출 진단이며 경로 통과/실제 조향 성립을 의미하지 않음.',source_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),counts=stats,held_previous_steer_range_rad=[min(steers),max(steers)],probes=probes)
(A/'result.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf8');print(json.dumps(data,ensure_ascii=True))

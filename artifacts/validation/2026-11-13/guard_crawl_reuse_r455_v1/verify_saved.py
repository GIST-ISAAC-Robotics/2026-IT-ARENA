"""r451 국소 입력의 hook 전달 비교. 실제 전체 _select/plan 복원은 별도 주장하지 않는다."""
from pathlib import Path
import copy,hashlib,importlib,json,sys,time
B=Path(__file__).resolve().parent;S=B/'stage'
sys.path[:0]=[str(S/'src/arena_autonomy'),str(S/'src/arena_vehicle_interface'),str(B/'original')]
from arena_autonomy.numeric_kernels import configure
configure('python')
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.follow_trace import FollowConfig,MotionConfig,restore_call,freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.race_runtime_policy import released_plan as current
from arena_autonomy.race_runtime_policy.retained_recheck._present import supported
original=importlib.import_module('race_runtime_policy.released_plan')
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def differences(a,b,path='$'):
    if isinstance(a,dict) and isinstance(b,dict):
        if set(a)!=set(b):return [dict(path=path,actual_keys=list(a),expected_keys=list(b))]
        return sum((differences(a[k],b[k],path+'.'+k) for k in a),[])
    if isinstance(a,list) and isinstance(b,list):
        if len(a)!=len(b):return [dict(path=path,lengths=[len(a),len(b)])]
        return sum((differences(x,y,path+f'[{i}]') for i,(x,y) in enumerate(zip(a,b))),[])
    if type(a) is float or type(b) is float:
        if isinstance(a,(float,int)) and isinstance(b,(float,int)) and abs(a-b)<=1e-12:return []
    return [] if a==b else [dict(path=path,actual=a,expected=b)]
def main():
    out=B/'saved_v1';out.mkdir(exist_ok=False)
    inputs=B.parent/'guard_crawl_cost_r454_v1/bundle/inputs'
    source_hash={p.name:digest(p) for p in inputs.glob('*')}
    header=json.loads((inputs/'header.json').read_bytes());cfg=FollowConfig(**header['follow_config'])
    motion_cfg=MotionConfig(**header['motion_config'])
    expected={r['seq']:r for r in json.loads((inputs/'expected.json').read_bytes())}
    rows=[json.loads(line) for line in (inputs/'selected.jsonl').open()]
    original_select=ps.PassReleaseSelector._select;original_guard=ps.PassReleaseSelector._guard_curve
    original_view=ps.threat_view;results=[];began=time.monotonic()
    with (out/'rows.jsonl').open('x',encoding='utf8') as stream:
        for row in rows:
            data=row['data'];normal=data['output'][2]['normal'];evidence=normal['selector_guard_candidate']
            info,scan,history=restore_call(data['input'],motion_cfg)
            scene=motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],epoch=info['epoch'],
                rear_half_angle_deg=info['rear_half_angle_deg'],timing_policy=info.get('timing_policy','verified_only'))
            base=(evidence['baseline_speed_mps'],evidence['steering_rad'],{**normal,'reason':evidence['baseline_reason']})
            parallel=(*data['output'][:2],{'reason':evidence['trigger']});counts=[];states=[];outputs=[];errors=[]
            for policy in (original,current):
                owner=ps.PassReleaseSelector(cfg);owner.sides=copy.deepcopy(normal['selector_sides'])
                owner.guard_anchor={name:entry['anchor_m'] for name,entry in evidence['guarded_sides'].items()}
                owner.last_choice=normal['selector_choice'];before=copy.deepcopy((owner.sides,owner.guard_anchor,owner.last_choice))
                counted=[]
                def view(*args):
                    value=original_view(*args);counted.append(value);return value
                def local(owner,scene,baseline,parallel,speed,previous,dt,*,raw_scan=None):
                    # 원 trace의 갱신 후 국소 상태를 그대로 사용한다. 없는 이전 전체 상태를 만들지 않는다.
                    value=ps.threat_view(scene,baseline[2]['path_points'],owner.lat)
                    prior=owner._guard_curve(scene,value,baseline,parallel,speed,previous,dt,
                        scene.t_ref_s-scene.source_first_s,{},raw_scan)
                    errors.extend(differences(freeze(prior[2]['selector_guard_candidate']),evidence,'$.original.guard'))
                    errors.extend(differences(freeze(prior[2]['selector_guard_speed_recheck']),
                        normal['selector_guard_speed_recheck'],'$.original.retry'))
                    return prior[0],prior[1],{**normal,**prior[2]}
                ps.PassReleaseSelector._select=local;ps.threat_view=view
                measured_token=policy._MEASURED.set(info['speed_mps'])
                try:
                    with policy.enabled():
                        result=owner._select(scene,base,parallel,abs(info['speed_mps']),
                            evidence['previous_steering_rad'],info['dt_s'],raw_scan=scan)
                finally:
                    policy._MEASURED.reset(measured_token)
                    ps.PassReleaseSelector._select=original_select;ps.threat_view=original_view
                assert ps.PassReleaseSelector._guard_curve is original_guard
                counts.append(len(counted));outputs.append(freeze(result))
                states.append((owner.sides,owner.guard_anchor,owner.last_choice)==before)
            errors+=differences(outputs[1],expected[row['seq']]['adapter_result'],'$.r451')
            if outputs[0]!=outputs[1]:errors.append(dict(path='$.old_vs_stage',exact=False))
            support=supported((outputs[1][0],outputs[1][1],{**data['output'][2],'normal':outputs[1][2]}))
            assert counts==[2,1] and all(states) and support
            record=dict(seq=row['seq'],passed=not errors,differences=errors,threat_view_calls=counts,
                original_stage_output_exact=outputs[0]==outputs[1],selector_state_unchanged=states,
                current_dispatch_supported=support,output=outputs[1])
            results.append(record);stream.write(json.dumps(record,allow_nan=False)+'\n');stream.flush()
            if errors:break
    changed=[p.name for p in inputs.glob('*') if digest(p)!=source_hash[p.name]]
    modules={k:str(Path(v.__file__).resolve()) for k,v in sys.modules.items()
        if k.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(v,'__file__',None)}
    external={k:v for k,v in modules.items() if not Path(v).is_relative_to(S)}
    report=dict(passed=len(results)==104 and all(r['passed'] for r in results) and not changed and not external,
        rows=len(results),old_stage_exact=sum(r['original_stage_output_exact'] for r in results),
        old_threat_view_calls=sum(r['threat_view_calls'][0] for r in results),
        stage_threat_view_calls=sum(r['threat_view_calls'][1] for r in results),
        candidate_accepted=sum(r['output'][0]>.0 for r in results),current_supported=sum(r['current_dispatch_supported'] for r in results),
        input_hashes=source_hash,input_changes=changed,external_project_imports=external,module_paths=modules,
        wall_s=time.monotonic()-began,backend='python',full_plan_replayed=False,actual_select_state_replay=False,
        scope='r451 recorded post-update local GUARD state through _guard_curve hook; real _select wiring covered in separate synthetic tests')
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('module_paths','input_hashes')}))
    return 0 if report['passed'] else 2
if __name__=='__main__':raise SystemExit(main())

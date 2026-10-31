"""실주행 저장 입력에서 후보 호출 여부와 OFF 차이를 확인한다. 새 주행 아님."""
from dataclasses import replace
import hashlib,json,zipfile
from pathlib import Path
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
INPUT=BASE.parent/'g2_ready_reference_gazebo_v1/run2/follow_decisions.jsonl'
def sha(path):
    with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def main():
    out=BASE/'run1';out.mkdir(exist_ok=False);digest=sha(INPUT)
    sources=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md']
    sources.extend(p for directory in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface') for p in (ROOT/directory).glob('*.py'))
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in sources}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in hashes:z.write(ROOT/p,p)
    calls=matched=same=fallback_calls=fallback_lookups=0;changes=[];uses=[];footer=None;per_call=[]
    for seq,line in enumerate(INPUT.open()):
        event=json.loads(line);assert event['seq']==seq and footer is None
        kind,data=event['kind'],event['data']
        if seq==0:
            assert kind=='header' and data['schema']==5
            config=FollowConfig(**data['follow_config']);execution=HandoffExecutionConfig.from_mapping(data['execution_config'])
            on=BypassHandoffFollow(config,execution,allow_simulation_trial=True)
            off=BypassHandoffFollow(config,replace(execution,initial_reference_policy='off'),allow_simulation_trial=True)
            motion=MotionConfig(**data['motion_config']);helper=on._initial_reference
            assert helper is not None
            original_reference=helper.reference
            def observed_reference(scene,cfg,monitor):
                result=original_reference(scene,cfg,monitor)
                if result is helper.proposed and result[0] is not None and helper.original[0] is None:
                    per_call.append(dict(reason=result[1].get('reason'),ready=monitor.result.get('ready'),observed=monitor.result.get('observed'),moving=monitor.motion_evidence()))
                return result
            helper.reference=observed_reference
        elif kind=='reset':on.reset();off.reset()
        elif kind=='call':
            per_call.clear();actual=freeze(replay_call(on,data['input'],motion));alternative=freeze(replay_call(off,data['input'],motion));calls+=1
            exact=equal_output(comparable_decision(actual),comparable_decision(data['output']));matched+=exact
            identical=equal_output(comparable_decision(actual),comparable_decision(alternative));same+=identical
            now=data['input']['reference_s'];fallback_calls+=bool(per_call);fallback_lookups+=len(per_call)
            if per_call:uses.append(dict(time_s=now,lookups=list(per_call),state=actual[2].get('handoff_state')))
            if not identical:changes.append(dict(time_s=now,on_command=actual[:2],off_command=alternative[:2],on_state=actual[2].get('handoff_state'),off_state=alternative[2].get('handoff_state'),on_reason=actual[2].get('reason'),off_reason=alternative[2].get('reason')))
        elif kind=='footer':footer=data
        else:raise ValueError('unexpected event')
    assert footer and footer['accepted']==footer['written']==seq and not footer['worker_alive']
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    unchanged=all(sha(ROOT/p)==h for p,h in hashes.items())
    summary=dict(passed=calls==matched and sha(INPUT)==digest and unchanged,input=INPUT.relative_to(ROOT).as_posix(),input_sha256=digest,input_unchanged=sha(INPUT)==digest,source_unchanged=unchanged,source_sha256=hashes,calls=calls,recorded_output_same=matched,off_full_decision_same=same,actual_fallback_calls=fallback_calls,actual_fallback_lookups=fallback_lookups,changes=changes,uses=uses,scope='same recorded input; OFF counterfactual is not a new trajectory or a causal repeatability trial')
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('source_sha256','changes','uses')}))
    raise SystemExit(0 if summary['passed'] else 1)
if __name__=='__main__':main()

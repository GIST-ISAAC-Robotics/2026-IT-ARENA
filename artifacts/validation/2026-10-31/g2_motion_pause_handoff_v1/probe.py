"""같은 저장 센서 입력에서 기존 제어기 재현과 비연결 후보를 대조한다."""
from pathlib import Path
from unittest.mock import patch
import hashlib
import json
import zipfile
from candidate import CandidateFollow
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]


def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def compare(path,output,prefix=None):
    digest=sha(path);calls=matched=changed=new_zero=overspeed_holds=illegal_positive=negative_positive=0
    first=None;transitions=[];footer=None
    with (output/(path.parent.parent.name+'_changes.jsonl')).open('x') as changes:
        for seq,line in enumerate(path.open()):
            event=json.loads(line);assert event['seq']==seq and footer is None
            kind,data=event['kind'],event['data']
            if seq==0:
                assert kind=='header' and data['schema']==5
                cfg=FollowConfig(**data['follow_config']);execution=HandoffExecutionConfig.from_mapping(data['execution_config'])
                motion=MotionConfig(**data['motion_config'])
                original=BypassHandoffFollow(cfg,execution,allow_simulation_trial=True)
                candidate=CandidateFollow(cfg,execution,allow_simulation_trial=True)
            elif kind=='reset':original.reset();candidate.reset()
            elif kind=='call':
                t=data['input']['reference_s']
                if prefix is not None and t>prefix:continue
                before=freeze(replay_call(original,data['input'],motion))
                after=freeze(replay_call(candidate,data['input'],motion));calls+=1
                matched+=equal_output(comparable_decision(before),comparable_decision(data['output']))
                diff=not equal_output(before[:2],after[:2]);changed+=diff;new_zero+=after[0]==0<before[0]
                detail=after[2];bypass=detail.get('bypass') or {};state=detail.get('handoff_state')
                held=bypass.get('reason')=='entry_overspeed_motion_hold';overspeed_holds+=held
                measured=(detail.get('normal') or {}).get('measured_speed_mps')
                negative_positive+=isinstance(measured,(int,float)) and measured<0 and after[0]>0
                illegal_positive+=held and after[0]!=0
                if not transitions or transitions[-1]['state']!=state:
                    transitions.append(dict(t=t,state=state,reason=detail.get('reason')))
                if diff or held:
                    row=dict(t=t,before=before[:2],after=after[:2],reason=detail.get('reason'),state=state,
                        bypass_reason=bypass.get('reason'),overspeed_hold=held,
                        measured=(detail.get('normal') or {}).get('measured_speed_mps'),
                        retained=bypass.get('geometry_retained'),forward_confirmations=bypass.get('forward_confirmations'))
                    changes.write(json.dumps(row)+'\n')
                    if first is None:first=row
            elif kind=='footer':footer=data
            else:raise ValueError('unknown event')
    assert footer and footer['accepted']==footer['written']==seq and not footer['worker_alive']
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    return dict(input=path.relative_to(ROOT).as_posix(),input_sha256=digest,input_unchanged=sha(path)==digest,
        prefix_s=prefix,calls=calls,baseline_reproduced=matched,command_changes=changed,new_zero_calls=new_zero,
        overspeed_hold_calls=overspeed_holds,positive_during_overspeed_hold=illegal_positive,positive_during_negative=negative_positive,first_change=first,
        candidate_transitions=transitions,footer_checked=True)


if __name__=='__main__':
    output=BASE/'run1';output.mkdir(exist_ok=False)
    files=[*BASE.glob('*.py'),BASE/'PLAN.md',*BASE.glob('*.sh'),BASE.parent/'g2_overspeed_hold_v1/candidate.py',ROOT/'tests/test_bypass_signed_hold_trial.py',ROOT/'scripts/probe_bypass_entry_sequence.py']
    files.extend(p for folder in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface') for p in (ROOT/folder).glob('*.py'))
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for relative in hashes:archive.write(ROOT/relative,relative)
    results=[compare(BASE.parent/'g2_restart_during_bypass_v1/run1/follow_decisions.jsonl',output,40.),
        compare(BASE.parent/'g2_level_surface_v1/run3/follow_decisions.jsonl',output,40.),
        compare(BASE.parent/'g1_forward_same_path_gazebo_v1/run1/follow_decisions.jsonl',output)]
    checks=dict(source_unchanged=all(sha(ROOT/p)==v for p,v in hashes.items()),
        baseline_exact=all(r['baseline_reproduced']==r['calls'] for r in results),
        inputs_unchanged=all(r['input_unchanged'] for r in results),
        no_positive_overspeed=all(r['positive_during_overspeed_hold']==0 for r in results),
        no_positive_negative=all(r['positive_during_negative']==0 for r in results),
        no_added_zero=all(r['new_zero_calls']==0 for r in results),
        successful_course_commands_same=results[2]['command_changes']==0)
    result=dict(passed=all(checks.values()),checks=checks,results=results,source_sha256=hashes,
                scope='saved input comparison only; candidate not runtime enabled; no new driving evidence')
    (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'}))
    raise SystemExit(0 if result['passed'] else 1)

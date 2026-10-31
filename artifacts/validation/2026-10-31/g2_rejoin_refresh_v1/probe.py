from pathlib import Path
from dataclasses import replace
import hashlib,json,zipfile
from candidate import CandidateFollow
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]


def sha(p):
    with p.open('rb') as s:return hashlib.file_digest(s,'sha256').hexdigest()


def compare(path,output,prefix=None):
    digest=sha(path);calls=matched=changes=added_zero=opened=refreshes=negative_positive=0;footer=None
    transitions=[];max_steering_before=max_steering_after=0.
    with (output/(path.parent.parent.name+'_changes.jsonl')).open('x') as log:
        for seq,line in enumerate(path.open()):
            e=json.loads(line);assert e['seq']==seq and footer is None
            kind,d=e['kind'],e['data']
            if seq==0:
                assert kind=='header' and d['schema']==5
                cfg=FollowConfig(**d['follow_config']);ex=HandoffExecutionConfig.from_mapping(d['execution_config'])
                motion=MotionConfig(**d['motion_config'])
                before=BypassHandoffFollow(cfg,ex,allow_simulation_trial=True)
                # 기존 성공 기록에서도 같은 운동 대기 연결을 기준으로 새 제안만 비교한다.
                after=CandidateFollow(cfg,replace(ex,motion_pause_policy='bounded_motion_pause_v1'),allow_simulation_trial=True)
            elif kind=='reset':before.reset();after.reset()
            elif kind=='call':
                t=d['input']['reference_s']
                if prefix is not None and t>prefix:continue
                a=freeze(replay_call(before,d['input'],motion));b=freeze(replay_call(after,d['input'],motion));calls+=1
                matched+=equal_output(comparable_decision(a),comparable_decision(d['output']))
                change=not equal_output(a[:2],b[:2]);changes+=change
                added_zero+=a[0]>0 and b[0]==0;opened+=a[0]==0 and b[0]>0
                detail=b[2];guide=detail.get('bypass') or {};normal=detail.get('normal') or {}
                negative_positive+=normal.get('measured_speed_mps',0)<0 and b[0]>0
                refreshes+=len(after.last_refresh_calls)
                max_steering_before=max(max_steering_before,abs(a[1]));max_steering_after=max(max_steering_after,abs(b[1]))
                state=detail.get('handoff_state')
                if not transitions or transitions[-1]['state']!=state:transitions.append(dict(t=t,state=state,reason=detail.get('reason')))
                if change or after.last_refresh_calls:
                    log.write(json.dumps(dict(t=t,before=a[:2],after=b[:2],state=state,bypass_reason=guide.get('reason'),
                        underlying=guide.get('underlying_reason'),refresh=after.last_refresh_calls,
                        phase=after.bypass.guide.phase,forward_confirmations=after.bypass.guide.forward_confirmations))+'\n')
            elif kind=='footer':footer=d
            else:raise ValueError(kind)
    assert footer and footer['accepted']==footer['written']==seq and not footer['worker_alive']
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    return dict(input=path.relative_to(ROOT).as_posix(),input_sha256=digest,input_unchanged=sha(path)==digest,
        calls=calls,baseline_exact=matched,command_changes=changes,new_zero_calls=added_zero,opened_calls=opened,
        refresh_calls=refreshes,positive_during_negative=negative_positive,transitions=transitions,
        max_abs_steering_before=max_steering_before,max_abs_steering_after=max_steering_after,footer_checked=True)


if __name__=='__main__':
    output=BASE/'run1';output.mkdir(exist_ok=False)
    files=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md',*(ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')]
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in hashes:z.write(ROOT/p,p)
    results=[compare(BASE.parent/'g2_motion_pause_gazebo_v1/run1/follow_decisions.jsonl',output,40.),
        compare(BASE.parent/'g1_forward_same_path_gazebo_v1/run1/follow_decisions.jsonl',output)]
    checks=dict(baseline_exact=all(r['baseline_exact']==r['calls'] for r in results),
        no_positive_negative=all(r['positive_during_negative']==0 for r in results),
        inputs_unchanged=all(r['input_unchanged'] for r in results),source_unchanged=all(sha(ROOT/p)==h for p,h in hashes.items()))
    summary=dict(integrity_passed=all(checks.values()),checks=checks,results=results,source_sha256=hashes,
        scope='saved input proposal comparison only; more open calls or smaller steering is not a driving pass')
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k!='source_sha256'}))
    raise SystemExit(0 if summary['integrity_passed'] else 1)

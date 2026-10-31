"""v1 원본 재현과 v2의 같은 입력 판단 비교. 실제 궤적 변경을 주장하지 않는다."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import zipfile
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
CASES=('g2_relative_yield_gazebo_v1/run2','g2_relative_yield_surface_v1/run1')


def sha(p):
    with p.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def compare(relative,output):
    path=BASE.parent/relative/'follow_decisions.jsonl';before=sha(path)
    calls=exact=changed=negative_positive=0;changes=[];selected=[];footer=None
    for seq,line in enumerate(path.open(encoding='utf-8')):
        event=json.loads(line);assert event['seq']==seq and footer is None
        kind,d=event['kind'],event['data']
        if seq==0:
            assert kind=='header' and d['schema']==5
            cfg=FollowConfig(**d['follow_config']);ex=HandoffExecutionConfig.from_mapping(d['execution_config'])
            assert ex.relative_yield_policy=='bounded_relative_yield_v1'
            motion=MotionConfig(**d['motion_config'])
            old=BypassHandoffFollow(cfg,ex,allow_simulation_trial=True)
            new=BypassHandoffFollow(cfg,replace(ex,relative_yield_policy='bounded_relative_yield_v2'),allow_simulation_trial=True)
        elif kind=='reset':old.reset();new.reset()
        elif kind=='call':
            a=freeze(replay_call(old,d['input'],motion));b=freeze(replay_call(new,d['input'],motion));calls+=1
            exact+=equal_output(comparable_decision(a),comparable_decision(d['output']))
            changed+=not equal_output(a[:2],b[:2])
            speed=(b[2].get('normal') or {}).get('measured_speed_mps')
            negative_positive+=isinstance(speed,(int,float)) and speed<0 and b[0]>0
            if not equal_output(a[:2],b[:2]):
                changes.append(dict(t=d['input']['reference_s'],old=a[:2],new=b[:2],
                    old_reason=(a[2].get('bypass') or {}).get('reason'),
                    new_reason=(b[2].get('bypass') or {}).get('reason'),
                    relative_yield=b[2].get('relative_yield')))
            if 18.7<=d['input']['reference_s']<=20.4:
                selected.append(dict(t=d['input']['reference_s'],old=a[:2],new=b[:2],
                    phase=(b[2].get('bypass') or {}).get('entry_phase'),
                    relative_yield=b[2].get('relative_yield')))
        elif kind=='footer':footer=d
        else:raise ValueError(kind)
    assert footer and footer['accepted']==footer['written']==seq and not footer['worker_alive']
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    key=relative.split('/')[0]
    (output/(key+'_changes.json')).write_text(json.dumps(freeze(changes),indent=2),encoding='utf-8')
    (output/(key+'_pause_window.json')).write_text(json.dumps(freeze(selected),indent=2),encoding='utf-8')
    return dict(input=str(path.relative_to(ROOT)),input_sha256=before,input_unchanged=sha(path)==before,
        calls=calls,v1_exact=exact,command_changes=changed,negative_input_positive_commands=negative_positive,
        first_change_s=changes[0]['t'] if changes else None,last_change_s=changes[-1]['t'] if changes else None)


if __name__=='__main__':
    output=BASE/'saved1';output.mkdir(exist_ok=False)
    paths=list(BASE.glob('*.py'))+list(BASE.glob('*.sh'))+[BASE/'README.md',ROOT/'config/tests/g2_relative_yield_r2_sha256.json']
    paths+=[p for p in (ROOT/'src').rglob('*.py') if '__pycache__' not in p.parts]
    paths+=list((ROOT/'tests').glob('test_bypass*.py'))
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in paths}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in hashes:archive.write(ROOT/name,name)
    rows=[compare(case,output) for case in CASES]
    same=all(sha(ROOT/name)==value for name,value in hashes.items())
    result=dict(scope='same recorded inputs; v2 trajectory not simulated',source_sha256=hashes,source_unchanged=same,
        cases=rows,passed=same and all(r['input_unchanged'] and r['calls']==r['v1_exact']
            and not r['negative_input_positive_commands'] for r in rows))
    (output/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'}))
    raise SystemExit(0 if result['passed'] else 1)

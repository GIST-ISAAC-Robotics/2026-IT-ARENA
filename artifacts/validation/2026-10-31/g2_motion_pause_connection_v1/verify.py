"""과거 출력 OFF 재현 및 변경 전 보존된 후보 명령과 ON 연결 결과를 대조한다."""
from dataclasses import replace
import hashlib,json,zipfile
from pathlib import Path
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
SAVED=BASE.parent/'g2_motion_pause_handoff_v1/run1'


def sha(p):
    with p.open('rb') as s:return hashlib.file_digest(s,'sha256').hexdigest()


def check(row):
    path=ROOT/row['input'];assert sha(path)==row['input_sha256']
    changes=SAVED/(path.parent.parent.name+'_changes.jsonl');changes_sha=sha(changes)
    cached=[json.loads(s) for s in changes.read_text().splitlines()]
    expected={r['t']:r['after'] for r in cached};assert len(expected)==len(cached)
    calls=off_match=on_match=0;footer=None;seen=set()
    for seq,line in enumerate(path.open()):
        e=json.loads(line);assert e['seq']==seq and footer is None
        kind,d=e['kind'],e['data']
        if seq==0:
            assert kind=='header' and d['schema']==5
            cfg=FollowConfig(**d['follow_config']);ex=HandoffExecutionConfig.from_mapping(d['execution_config'])
            assert ex.motion_pause_policy=='off'
            motion=MotionConfig(**d['motion_config'])
            off=BypassHandoffFollow(cfg,ex,allow_simulation_trial=True)
            on=BypassHandoffFollow(cfg,replace(ex,motion_pause_policy='bounded_motion_pause_v1'),allow_simulation_trial=True)
        elif kind=='reset':off.reset();on.reset()
        elif kind=='call':
            t=d['input']['reference_s']
            if row['prefix_s'] is not None and t>row['prefix_s']:continue
            a=freeze(replay_call(off,d['input'],motion));b=freeze(replay_call(on,d['input'],motion));calls+=1
            off_match+=equal_output(comparable_decision(a),comparable_decision(d['output']))
            on_match+=equal_output(b[:2],expected.get(t,d['output'][:2]))
            if t in expected:seen.add(t)
        elif kind=='footer':footer=d
        else:raise ValueError(kind)
    assert footer and footer['accepted']==footer['written']==seq and not footer['worker_alive']
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    return dict(input=row['input'],calls=calls,off_exact=off_match,on_cached_commands_exact=on_match,
        cached_events=len(expected),cached_events_seen=len(seen),footer_checked=True,
        unchanged=sha(path)==row['input_sha256'] and sha(changes)==changes_sha,
        passed=calls==row['calls']==off_match==on_match and seen==set(expected))


if __name__=='__main__':
    output=BASE/'run1';output.mkdir(exist_ok=False)
    files=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md',SAVED/'summary.json',
        ROOT/'config/tests/g2_motion_pause_r1_sha256.json']
    files.extend(p for folder in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface') for p in (ROOT/folder).glob('*.py'))
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in hashes:z.write(ROOT/p,p)
    saved=json.loads((SAVED/'summary.json').read_text());assert saved['passed']
    results=[check(row) for row in saved['results']]
    immutable=all(sha(ROOT/p)==v for p,v in hashes.items())
    result=dict(passed=immutable and all(r['passed'] and r['unchanged'] for r in results),
        source_unchanged=immutable,results=results,source_sha256=hashes)
    (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'}))
    raise SystemExit(0 if result['passed'] else 1)

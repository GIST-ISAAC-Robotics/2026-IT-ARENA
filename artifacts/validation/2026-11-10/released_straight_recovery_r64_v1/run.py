"""r64과 직전 실패와 같은 코너 각본 1회. 설정·판정 변경 없이 검사한다."""
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import zipfile

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
OLD=ROOT/'artifacts/validation/2026-11-08/observed_corner_continuity_r48_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert not (A/'run1').exists() and not (A/'registered.json').exists()
    previous=A.parent/'corner_parallel_continuity_v1'
    for name in ('new_track_course.json','geometry.json'):
        assert sha(A/name)==sha(previous/name)
    assert (A/'tests.exit').read_text().strip()=='0'
    assert (A/'replay.exit').read_text().strip()=='0'
    replay=json.loads((A/'replay/summary.json').read_bytes())
    assert replay['source_unchanged']
    for path,digest in replay['sources'].items():assert sha(ROOT/path)==digest
    assert replay['runs']['corner']['counts']['command_changes']>0
    assert replay['runs']['corner']['accepted']['released_straight_presteer']>0
    for name in ('restart','moving','combined'):
        assert replay['runs'][name]['counts'].get('command_changes',0)==0
    assert all(row['input_unchanged'] and row['counts']['before_recorded_mismatch']==0 for row in replay['runs'].values())
    g=json.loads((A/'geometry.json').read_bytes())
    assert g['initial_ego_actor_gap_m']>=.08 and g['route']['actor_max_off_wheel_centers']<=2
    selected=next(x for x in g['candidates'] if x['approach_speed_mps']==g['selected_approach_mps'])
    assert selected['longest_parallel_s']>=.5 and selected['stored_corner_minimum_body_gap_m']>=.08
    eval_lock=json.loads((OLD/'evaluation_source.json').read_bytes())
    for path,expected in eval_lock.items():assert sha(ROOT/path)==expected
    argv=list(json.loads((OLD/'registered.json').read_bytes())['args'])
    argv[argv.index('--config')+1]=(A/'new_track_course.json').relative_to(ROOT).as_posix()
    argv[argv.index('--output')+1]=(A/'run1').relative_to(ROOT).as_posix()
    sources=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
    sources.update(x.relative_to(ROOT).as_posix() for x in A.iterdir() if x.suffix in ('.py','.sh','.json'))
    sources.update(eval_lock)
    sources=sorted(p for p in sources if (ROOT/p).is_file())
    hashes={p:sha(ROOT/p) for p in sources}
    record=dict(registered_at_utc=datetime.now(timezone.utc).isoformat(),
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        parent_args=(OLD/'registered.json').relative_to(ROOT).as_posix(),args=argv,
        scope='r64 same failed corner schedule, configuration and gates; no runtime truth fed to ego controller',
        previous_run=(previous/'registered.json').relative_to(ROOT).as_posix(),
        previous_registration_sha256=sha(previous/'registered.json'),
        original_evaluator=eval_lock, sources=hashes,
        gates=dict(corner_parallel_s=.5,minimum_gap_m=.08,maximum_low_speed_s=2.5,
                   minimum_progress_m=10.5,maximum_offroad_wheels=2,
                   required=['no_contact','curve_exit','STOP','clean_shutdown','unchanged_sources']))
    with zipfile.ZipFile(A/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for path in sources:z.write(ROOT/path,path)
    record['snapshot_sha256']=sha(A/'sources.zip')
    (A/'registered.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    # 환경의 실제 import 위치도 확인한다. 오래된 install 사본으로 비교하지 않는다.
    names=('local_follow','pass_release_trial','released_straight_recovery','bypass_observed_return','bypass_handoff','g1_g2_node')
    runtime=[]
    for name in names:
        spec=importlib.util.find_spec('arena_autonomy.'+name)
        actual=Path(spec.origin).resolve()
        expected=ROOT/'src/arena_autonomy/arena_autonomy'/f'{name}.py'
        assert actual==expected.resolve() and sha(actual)==hashes[expected.relative_to(ROOT).as_posix()]
        runtime.append(dict(module=name,path=str(actual),sha256=sha(actual)))
    (A/'runtime_imports.json').write_text(json.dumps(runtime,indent=2)+'\n')
    try:
        return subprocess.call([sys.executable,'-B',*argv],cwd=ROOT)
    finally:
        changed=[p for p,h in hashes.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
        integrity=dict(changed=changed,snapshot_unchanged=sha(A/'sources.zip')==record['snapshot_sha256'])
        (A/'postflight_integrity_run1.json').write_text(json.dumps(integrity,indent=2)+'\n')


if __name__=='__main__':raise SystemExit(main())

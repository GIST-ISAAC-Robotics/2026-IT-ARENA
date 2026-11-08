"""P22 제한 입구 시험 입력 등록. 원본/기존 시험 등록을 고치지 않는다."""
from pathlib import Path
import datetime
import hashlib
import json
import subprocess
import sys
import zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    assert not any((A/n).exists() for n in ('registered.json','source_snapshot.zip','run1'))
    args=json.loads(subprocess.check_output([sys.executable,'-B','scripts/run_current_solo.py',str(A/'run1'),
        '--print-args','--track','official_v2026_11_06','--local-sensor-config',
        'src/arena_description/config/b0390_c1_jr60p20_sim.yaml','--piecewise-wall','--standing-wall-reacquire'],cwd=ROOT,text=True))
    args.remove('--video')
    args[args.index('--max-sim-seconds')+1]='36'
    args += ['--p22-entry-trial','--target-progress-m','100']
    files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
    files|={p.relative_to(ROOT).as_posix() for p in A.glob('*') if p.suffix in ('.py','.sh','.md')}
    files|={'tests/test_p22_trial.py','tests/test_source_provenance_r0_plus_b3.py'}
    files=sorted(f for f in files if (ROOT/f).is_file())
    contract='config/tests/p22_entry_wiring_r49_sha256.json'
    c=json.loads((ROOT/contract).read_bytes())
    for group in ('approved_changes','approved_evaluation_scripts'):
        assert all(sha(ROOT/f)==v['after'] for f,v in c[group].items())
    assert all(sha(ROOT/f)==v for f,v in c['new_dependencies'].items())
    with zipfile.ZipFile(A/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for f in files:z.write(ROOT/f,f)
    data=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),args=args,contract=contract,
        scope='one sensor-only solo P22 entry, no merge/lap benefit/traffic claim',
        evaluation_only=dict(entry_wall_x_m=5.635,entry_y_interval_m=[4.2,4.8],
                             whole_body_inside=True,heading_west_tolerance_deg=20.,max_offroad_wheels=2,
                             zero_collision=True,explicit_STOP_and_clean_shutdown=True),
        sources={f:sha(ROOT/f) for f in files},snapshot_sha256=sha(A/'source_snapshot.zip'))
    (A/'registered.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print('registered',len(files),'files; no simulator started')
if __name__=='__main__':main()

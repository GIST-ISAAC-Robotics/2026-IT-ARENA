"""기존 전방 합류 각본에 시설·오목 시각 상대를 결합한다."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile
from datetime import datetime, timezone

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
FAC=ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'
PREVIOUS=ROOT/'artifacts/validation/2026-11-08/cut_in_early_newcourse_v2'
sys.path[:0]=[str(FAC),str(ROOT/'scripts'),str(ROOT/'tests')]
import validate_g1_traffic as target
from traffic_posts_setup import configure


def main():
    geometry=json.loads((A/'geometry.json').read_bytes())
    assert geometry['passed']
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in geometry['sources'].items())
    folder=A/'close_cut_in';folder.mkdir(exist_ok=False)
    parent=PREVIOUS/'close_cut_in/registered.json'
    registration=json.loads(parent.read_bytes())
    args=[str(folder/'run1') if v=='OUTPUT' else v for v in registration['args']]
    index=args.index('--config')+1
    config=json.loads((ROOT/args[index]).read_bytes())
    config['actor']['visual_outline']='recessed_sides'
    path=folder/'recessed_config.json'
    path.write_text(json.dumps(config,indent=2)+'\n');args[index]=str(path)
    # 평가기는 이전 주행 당시 원본 그대로인지 대조한다.
    p=PREVIOUS/'goal_eval.py';rel=p.relative_to(ROOT).as_posix()
    assert hashlib.sha256(p.read_bytes()).hexdigest()==registration['sources'][rel]
    extra=[Path(__file__),A/'run.sh',A/'assess.py',path,p, A/'geometry.json',A/'geometry.py',
        ROOT/'config/tests/released_straight_restore_r65_sha256.json',
        *[FAC/n for n in ('traffic_posts_setup.py','traffic_facility_fixture.py',
                          'marker_posts_provisional.sdf','marker_posts_provisional.json','evaluate_traffic_posts.py')],
        *[ROOT/'artifacts/validation/2026-11-10/facility_traffic_stability_v1'/n
          for n in ('audit_recessed.py','analyze_continuity.py')],
        ROOT/'artifacts/validation/2026-11-08/medium_recessed_pass_v1/outline_moving.py',
        ROOT/'scripts/audit_actor_outline_returns.py']
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in extra}
    record=dict(registered_at_utc=datetime.now(timezone.utc).isoformat(),
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        parent=parent.relative_to(ROOT).as_posix(),args=args,extra_sources=hashes,
        gates=dict(original_cutin_goal_unchanged=True,original_report_preserved=True,
            facility_overlap_samples=0,required=['source_integrity','STOP','clean_shutdown','recessed_return_audit']),
        scope='same 9.5s front merge schedule with posts and visual concavity; current validated code, no speed increase')
    (folder/'registered.json').write_text(json.dumps(record,indent=2)+'\n')
    with zipfile.ZipFile(folder/'extra_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in extra:z.write(p,p.relative_to(ROOT))
    configure(target,folder/'stage')
    sys.argv=args
    try:return target.main()
    finally:
        changed=[p for p,h in hashes.items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        (folder/'extra_integrity.json').write_text(json.dumps(dict(changed=changed),indent=2)+'\n')


if __name__=='__main__':raise SystemExit(main())

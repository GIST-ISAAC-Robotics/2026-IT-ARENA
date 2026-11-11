"""r81 native 결측 구분 보존만 바꾼 동일 시설+C1 제한1회 등록."""
from datetime import datetime,timezone
import hashlib,json,subprocess,sys,zipfile
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
BASE=ROOT/'artifacts/validation/2026-11-11/native_traffic_r79_v1'
F=ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'
PROFILE=ROOT/'config/tests/lidar_uniform30_delay50_jitter20.json'
sys.path[:0]=[str(F),str(ROOT/'scripts'),str(ROOT/'tests')]
import validate_g1_traffic as target
from traffic_posts_setup import configure


def main():
    checks=json.loads((BASE/'posthoc_v2/combined_assessment.json').read_text(encoding='utf-8'))
    assert checks['gates']['native_input_verified'] and checks['gates']['posthoc_inputs_unchanged']
    assert json.loads((A/'checks.json').read_text(encoding='utf-8'))['passed']
    d=A/'two_actor_run1';d.mkdir(exist_ok=False)
    parent=BASE/'two_actor_run1/registered.json';record=json.loads(parent.read_bytes())
    args=list(record['args']);args[args.index('--output')+1]=str(d/'run1')
    assert args.count('--lidar-native-sampling-profile')==1
    # 같은 형상/각본 파일 바이트를 복사한다. 기존 등록/원시 결과는 변경하지 않는다.
    for flag,name in (('--config','recessed_config.json'),('--actor-overlay','additional_actors.json')):
        src=BASE/'two_actor_run1'/name;dst=d/name;dst.write_bytes(src.read_bytes())
        args[args.index(flag)+1]=str(dst)
    assert args[args.index('--lidar-test-profile')+1]==PROFILE.relative_to(ROOT).as_posix()
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
    files.update(p.relative_to(ROOT).as_posix() for p in A.glob('*') if p.is_file() and p.suffix in ('.py','.sh'))
    files.update(p.relative_to(ROOT).as_posix() for p in (F/'traffic_posts_setup.py',F/'traffic_facility_fixture.py',
        F/'traffic_posts_assess.py',F/'marker_posts_provisional.sdf',F/'marker_posts_provisional.json',
        ROOT/'artifacts/validation/2026-11-10/combined_recessed_facility_v1/audit_shapes.py',ROOT/'artifacts/validation/2026-11-08/masked_motion_speed_v1/assess.py',
        ROOT/'artifacts/validation/2026-11-08/medium_recessed_pass_v1/outline_moving.py',
        ROOT/'artifacts/validation/2026-11-10/facility_traffic_stability_v1/analyze_continuity.py',d/'recessed_config.json',d/'additional_actors.json'))
    hashes={name:sha(ROOT/name) for name in sorted(files) if (ROOT/name).is_file()}
    with zipfile.ZipFile(d/'extra_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name in hashes:z.write(ROOT/name,name)
    registration=dict(registered_at_utc=datetime.now(timezone.utc).isoformat(),
        parent=parent.relative_to(ROOT).as_posix(),parent_sha256=sha(parent),args=args,extra_sources=hashes,
        snapshot_sha256=sha(d/'extra_sources.zip'),
        gates=dict(progress_m=44.64,maximum_low_speed_s=2.5,minimum_gap_m=.08,maximum_offroad_wheels=2,
            contacts=0,facility_overlaps=0,required=['STOP','clean_shutdown','source_unchanged','injection_verified','actor_structure_unchanged']),
        scope='same r79 controller,81T,IMU,noise/delay,facility,actors,native templates; only codec preserves positive infinity separately from NaN; r80 candidate not adopted; no hardware timing proof')
    (d/'registered.json').write_text(json.dumps(registration,indent=2)+'\n')
    configure(target,d/'stage');sys.argv=args
    try:return target.main()
    finally:
        result=dict(changed=[p for p,h in hashes.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h],
            snapshot_unchanged=sha(d/'extra_sources.zip')==registration['snapshot_sha256'])
        (d/'extra_integrity.json').write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':raise SystemExit(main())

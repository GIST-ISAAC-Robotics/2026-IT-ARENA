"""두 앞차·시설·마커 누락에 기존 오목한 상대 형상을 결합한다. 속도/판정 불변."""
import hashlib
import json
import subprocess
import sys
import zipfile
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PREPARED = ROOT / 'artifacts/validation/2026-11-09/signal_posts_markers_v1'
sys.path[:0] = [str(PREPARED), str(ROOT/'scripts'), str(ROOT/'tests')]
import validate_g1_traffic as target
from traffic_posts_setup import configure


def main():
    assert json.loads((HERE/'setup_check.json').read_bytes())['passed']
    # 미채택 r61은 제거했고 실제 C6에서 검증한 선택기 바이트를 사용한다.
    p='src/arena_autonomy/arena_autonomy/pass_release_trial.py'
    c6=ROOT/'artifacts/validation/2026-11-10/facility_traffic_stability_v1/stationary_r60/run1/report.json'
    assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==json.loads(c6.read_bytes())['source_sha256'][p]
    assert 'passed' in (HERE/'restore_checks.log').read_text() and (HERE/'restore_checks.exit').read_text().strip()=='0'
    folder = HERE/'two_actor_run1'
    folder.mkdir(exist_ok=False)
    parent = ROOT/'artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1/two_actor_course_lap/registered.json'
    args = [str(folder/'run1') if v == 'OUTPUT' else v
            for v in json.loads(parent.read_text(encoding='utf8'))['args']]
    i=args.index('--config')+1
    config=json.loads((ROOT/args[i]).read_text())
    config['actor']['visual_outline']='recessed_sides'
    path=folder/'recessed_config.json'
    path.write_text(json.dumps(config,indent=2)+'\n')
    args[i]=str(path)
    i=args.index('--actor-overlay')+1
    overlay=json.loads((ROOT/args[i]).read_text())
    for actor in overlay['additional_actors']:actor['visual_outline']='box'
    overlay_path=folder/'additional_actors.json'
    overlay_path.write_text(json.dumps(overlay,indent=2)+'\n')
    args[i]=str(overlay_path)
    extra = [Path(__file__), HERE/'run.sh', HERE/'assess.py',
             path, overlay_path, HERE/'audit_shapes.py',
             ROOT/'artifacts/validation/2026-11-08/medium_recessed_pass_v1/outline_moving.py',
             ROOT/'scripts/audit_actor_outline_returns.py',
             *[PREPARED/p for p in ('traffic_posts_setup.py', 'traffic_facility_fixture.py',
                'marker_posts_provisional.sdf', 'marker_posts_provisional.json', 'traffic_posts_assess.py')]]
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in extra}
    record = dict(registered_at_utc=datetime.now(timezone.utc).isoformat(),
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        args=args, parent_args=parent.relative_to(ROOT).as_posix(), extras=hashes,
        scope='two scripted 0.8mps actors, primary recessed visual, 60 simulation seconds, provisional posts and missing IDs20/30; not independent multi-car proof',
        shape_gates=dict(recessed_returns_minimum=2,unique_scans_minimum=2,
            more_recessed_than_old_box=True,time_shift_s=[-.01,.01],original_collision_unchanged=True),
        gates=dict(minimum_progress_m=44.64, maximum_low_speed_s=2.5, minimum_actor_gap_m=.08,
            maximum_offroad_wheels=2, post_overlap_samples=0, foot_overlap_samples=0,
            original_report_preserved=True,
            required=['input_integrity','STOP','clean_shutdown','no_contact','facility_xml']))
    (folder/'registered.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf8')
    with zipfile.ZipFile(folder/'extra_sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in extra: archive.write(p,p.relative_to(ROOT))
    configure(target,folder/'stage')
    sys.argv = args
    try:
        return target.main()
    finally:
        changed = [p for p,h in hashes.items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        (folder/'extra_integrity.json').write_text(json.dumps(dict(changed=changed),indent=2)+'\n',encoding='utf8')


if __name__ == '__main__':
    raise SystemExit(main())

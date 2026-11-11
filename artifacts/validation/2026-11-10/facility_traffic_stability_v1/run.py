"""11/9 준비한 시설 교통 시험을 11/10 별도 산출물로 실행한다."""
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
    folder = HERE/'two_actor_run1'
    folder.mkdir(exist_ok=False)
    parent = ROOT/'artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1/two_actor_course_lap/registered.json'
    args = [str(folder/'run1') if v == 'OUTPUT' else v
            for v in json.loads(parent.read_text(encoding='utf8'))['args']]
    extra = [Path(__file__), HERE/'run.sh', HERE/'assess.py',
             *[PREPARED/p for p in ('traffic_posts_setup.py', 'traffic_facility_fixture.py',
                'marker_posts_provisional.sdf', 'marker_posts_provisional.json', 'traffic_posts_assess.py')]]
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in extra}
    record = dict(registered_at_utc=datetime.now(timezone.utc).isoformat(),
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        args=args, parent_args=parent.relative_to(ROOT).as_posix(), extras=hashes,
        scope='two scripted 0.8mps actors, 60 simulation seconds, provisional posts and missing IDs20/30; no independent multi-car proof',
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

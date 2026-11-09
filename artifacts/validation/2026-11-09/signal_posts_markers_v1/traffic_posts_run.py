"""시설 전체+마커 누락+두 앞차 60초 대표. 실행 전 목적/판정을 등록한다."""
import hashlib,json,sys,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(R/'scripts'),str(R/'tests')]
import validate_g1_traffic as target
from traffic_posts_setup import configure

def main():
    folder=A/'traffic_all_posts';out=folder/'run1'
    assert not out.exists()
    folder.mkdir(exist_ok=False)
    parent=R/'artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1/two_actor_course_lap/registered.json'
    registered=json.loads(parent.read_text(encoding='utf8'))
    args=[str(out) if v=='OUTPUT' else v for v in registered['args']]
    extra=[Path(__file__),A/'traffic_posts_setup.py',A/'traffic_facility_fixture.py',
           A/'marker_posts_provisional.sdf',A/'marker_posts_provisional.json',A/'traffic_posts_assess.py']
    hashes={p.relative_to(R).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in extra}
    record=dict(args=args,parent_args=str(parent.relative_to(R)),extras=hashes,
        scope='two scripted .8mps actors, 60 simulation seconds, all provisional posts and blank20/30; no five autonomous car proof',
        gates=dict(minimum_progress_m=44.64,maximum_low_speed_s=2.5,minimum_actor_gap_m=.08,
                   maximum_offroad_wheels=2,post_overlap_samples=0,foot_overlap_samples=0,
                   original_report_preserved=True,required=['input_integrity','STOP','clean_shutdown','no_contact','facility_xml']))
    (folder/'registered.json').write_text(json.dumps(record,indent=2)+'\n')
    with zipfile.ZipFile(folder/'extra_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in extra:z.write(p,p.relative_to(R))
    configure(target,folder/'stage')
    sys.argv=args
    try:return target.main()
    finally:
        changed=[p for p,h in hashes.items() if hashlib.sha256((R/p).read_bytes()).hexdigest()!=h]
        (folder/'extra_integrity.json').write_text(json.dumps(dict(changed=changed),indent=2)+'\n')

if __name__=='__main__':raise SystemExit(main())

"""원판·직진 우선 후보의 2/3/5대 각 위치 범위 점검. 앞차 선출발이라는 조건부 이상 운동."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import yaml

from simulate_grid_departure import simulate
from probe_grid_follow import ROOT,CAMERA,VEHICLE,RUNTIME,registry,wall_sections,json_value
from audit_local_pursuit_road import road_polygon
from audit_grid_departure_motion import audit
from surface_contact_eval import VehicleGeometry
from shapely.geometry import MultiLineString


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):parser.error('validation output required')
    out.mkdir(parents=True,exist_ok=False)
    vehicle=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    slots=registry(json.loads((RUNTIME/'scene.json').read_text(encoding='utf-8')),(RUNTIME/'world.sdf').read_bytes(),vehicle)
    segments=wall_sections((RUNTIME/'world.sdf').read_bytes(),sensor['xyz_m'][2]);walls=MultiLineString(segments.tolist())
    road=road_polygon(RUNTIME/'world.sdf')
    geometry=VehicleGeometry(wheelbase_m=vehicle['drivetrain']['wheelbase_m'],track_width_m=vehicle['drivetrain']['track_width_m'])
    paths={Path(__file__),CAMERA,VEHICLE,RUNTIME/'world.sdf',RUNTIME/'scene.json'}
    paths.update((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    for m in tuple(sys.modules.values()):
        name=getattr(m,'__file__',None)
        if name:
            p=Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py':paths.add(p)
    frozen={p.relative_to(ROOT).as_posix():p.read_bytes() for p in paths}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for n,b in frozen.items():z.writestr(n,b)
    summaries=[]
    for count in (2,3,5):
        selected=slots[:count]
        for ego in selected:
            if count==5 and ego['index']==5:continue # run20 ON의 같은 조건을 재실행하지 않는다.
            result=simulate(ego,[s for s in selected if s['index']!=ego['index']],segments,road,vehicle,sensor,
                rounded=True,fault=False,straight_priority=True)
            result['summary'].update(grid_count=count,ego_grid_index=ego['index'])
            result['continuous_audit']=audit(result,selected,walls,road,geometry,ego_index=ego['index'])
            result['summary']['continuous_audit']=result['continuous_audit']
            result['summary']['scan_schedule_valid']=all(row['frame']==max(s['frame'] for s in result['scans']
                if round((s['first_s']+.0998+.002)*1e9)<=round(row['t_s']*1e9)) for row in result['calls'])
            name=f"grid{count}_ego{ego['index']}"
            case_dir=out/name;case_dir.mkdir()
            (case_dir/'report.json').write_text(json.dumps(json_value(result),ensure_ascii=False,allow_nan=False),encoding='utf-8')
            summaries.append(result['summary'])
            print(json.dumps(result['summary']),flush=True)
    unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items())
    integrity=unchanged and all(c['scan_schedule_valid'] and c['calls']==270 and c['continuous_audit']['passed'] and
        c['motion_input_failures']==0 and c['explicit_stop_checked'] and c['stopped_after_request'] for c in summaries)
    summary=dict(cases=summaries,sources_unchanged=unchanged,integrity_pass=integrity,
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},driving_authorized=False,
        reused_case='grid5_ego5: run20_straight_dynamics case 1; not rerun',
        scope='ego motion plus scripted ahead peers, rear peers fixed; not joint autonomous start or ROS/MCU/physical braking')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if integrity and len(summaries)==9 else 1


if __name__=='__main__':raise SystemExit(main())

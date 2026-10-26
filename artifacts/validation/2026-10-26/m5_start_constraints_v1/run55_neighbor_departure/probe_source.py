"""M5 앞쪽 두 칸: 같은 최신 후보에서 정지 옆줄과 선출발 옆줄의 이상 운동 대조."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import yaml
from shapely.geometry import MultiLineString
from simulate_grid_departure import simulate
from probe_grid_follow import ROOT,CAMERA,VEHICLE,RUNTIME,registry,wall_sections,json_value
from audit_local_pursuit_road import road_polygon
from audit_grid_departure_motion import audit
from surface_contact_eval import VehicleGeometry


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):parser.error('new validation output required')
    out.mkdir(parents=True,exist_ok=False)
    vehicle=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    slots=registry(json.loads((RUNTIME/'scene.json').read_text(encoding='utf-8')),(RUNTIME/'world.sdf').read_bytes(),vehicle)[:5]
    segments=wall_sections((RUNTIME/'world.sdf').read_bytes(),sensor['xyz_m'][2])
    walls=MultiLineString(segments.tolist());road=road_polygon(RUNTIME/'world.sdf')
    geometry=VehicleGeometry(wheelbase_m=vehicle['drivetrain']['wheelbase_m'],track_width_m=vehicle['drivetrain']['track_width_m'])
    paths={Path(__file__),CAMERA,VEHICLE,RUNTIME/'world.sdf',RUNTIME/'scene.json'}
    paths.update((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    for module in tuple(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            p=Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py':paths.add(p)
    frozen={p.relative_to(ROOT).as_posix():p.read_bytes() for p in paths}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for n,b in frozen.items():z.writestr(n,b)
    cases=[]
    for index,policy,fault in ((2,'ahead_only',False),(2,'ahead_or_other_lane',False),
                              (1,'ahead_only',False),(1,'ahead_or_other_lane',False),
                              (2,'ahead_or_other_lane',True)):
        ego=next(s for s in slots if s['index']==index)
        result=simulate(ego,[s for s in slots if s!=ego],segments,road,vehicle,sensor,
            rounded=True,straight_priority=True,defer_center_on_straight=True,
            peer_motion_policy=policy,fault=fault)
        result['continuous_audit']=audit(result,slots,walls,road,geometry,ego_index=index)
        summary=result['summary']
        summary.update(grid_count=5,ego_grid_index=index,continuous_audit=result['continuous_audit'])
        summary['scan_schedule_valid']=all(row['frame']==max(s['frame'] for s in result['scans']
            if round(s['delivered_s']*1e9)<=round(row['t_s']*1e9)) for row in result['calls'])
        name=f'grid5_ego{index}_{policy}'+('_missing' if fault else '')
        case_dir=out/name;case_dir.mkdir()
        (case_dir/'report.json').write_text(json.dumps(json_value(result),ensure_ascii=False,allow_nan=False),encoding='utf-8')
        cases.append(dict(name=name,**summary));print(json.dumps(cases[-1]),flush=True)
    unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items())
    integrity=unchanged and all(c['scan_schedule_valid'] and c['calls']==270 and c['continuous_audit']['passed'] and
        c['motion_input_failures']==0 and c['explicit_stop_checked'] and c['stopped_after_request'] and
        c['fault_positive_requests']==0 and c['fault_positive_safe']==0 for c in cases)
    summary=dict(cases=cases,sources_unchanged=unchanged,integrity_pass=integrity,
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
        scope='scripted neighboring lane moves first; same-lane rear stays fixed; not joint autonomous or physical start')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if integrity and len(cases)==5 else 1


if __name__=='__main__':raise SystemExit(main())

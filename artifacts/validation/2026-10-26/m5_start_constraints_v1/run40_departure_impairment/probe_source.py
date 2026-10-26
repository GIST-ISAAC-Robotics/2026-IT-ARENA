"""진행했던 M5 두 대 배치에 유계 거리 편향/수신 지연을 주입하는 제한 이상 운동."""
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
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):parser.error('new validation output required')
    out.mkdir(parents=True,exist_ok=False)
    vehicle=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    slots=registry(json.loads((RUNTIME/'scene.json').read_text(encoding='utf-8')),(RUNTIME/'world.sdf').read_bytes(),vehicle)[:2]
    ego=next(s for s in slots if s['index']==5)
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
    for name,bias,delay in (('minus30',-.03,0.),('plus30',.03,0.),('delay50',0.,.05),('minus30_delay50',-.03,.05)):
        result=simulate(ego,[s for s in slots if s!=ego],segments,road,vehicle,sensor,rounded=True,
            straight_priority=True,range_bias_m=bias,additional_scan_delay_s=delay)
        result['continuous_audit']=audit(result,slots,walls,road,geometry,ego_index=5)
        summary=result['summary'];summary['continuous_audit']=result['continuous_audit']
        summary['scan_schedule_valid']=all(row['frame']==max(s['frame'] for s in result['scans']
            if round(s['delivered_s']*1e9)<=round(row['t_s']*1e9)) for row in result['calls'])
        summary['expected_calls']=sum(round((.6018+i*.02)*1e9)>=round((.6018+delay)*1e9) for i in range(270))
        case_dir=out/('grid2_ego5_'+name);case_dir.mkdir()
        (case_dir/'report.json').write_text(json.dumps(json_value(result),ensure_ascii=False,allow_nan=False),encoding='utf-8')
        cases.append(dict(name=name,**summary));print(json.dumps(cases[-1]),flush=True)
    unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items())
    integrity=unchanged and all(c['scan_schedule_valid'] and c['calls']==c['expected_calls'] and
        c['continuous_audit']['passed'] and c['motion_input_failures']==0 and c['explicit_stop_checked'] and
        c['stopped_after_request'] for c in cases)
    summary=dict(cases=cases,sources_unchanged=unchanged,integrity_pass=integrity,
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
        nominal_reference='run31_grid_coverage/data/grid2_ego5/report.json; no nominal rerun',
        scope='scripted lead plus ideal ego, sensor impairments only; no MCU/ROS/physics/deadline proof')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if integrity and len(cases)==4 else 1


if __name__=='__main__':raise SystemExit(main())

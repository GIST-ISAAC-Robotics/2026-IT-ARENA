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
    parser.add_argument('--moving-dropout-only',action='store_true',
        help='run55 칸2 명목과 같은 조건에서 이미 출발한 뒤 프레임30–33 중앙 누락 1회')
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
    definitions=((2,'ahead_or_other_lane',True),) if args.moving_dropout_only else ((2,'ahead_only',False),(2,'ahead_or_other_lane',False),
                              (1,'ahead_only',False),(1,'ahead_or_other_lane',False),
                              (2,'ahead_or_other_lane',True))
    for index,policy,fault in definitions:
        ego=next(s for s in slots if s['index']==index)
        result=simulate(ego,[s for s in slots if s!=ego],segments,road,vehicle,sensor,
            rounded=True,straight_priority=True,defer_center_on_straight=True,
            peer_motion_policy=policy,fault=fault,fault_frames=(30,34) if args.moving_dropout_only else (18,22))
        result['continuous_audit']=audit(result,slots,walls,road,geometry,ego_index=index)
        summary=result['summary']
        summary.update(grid_count=5,ego_grid_index=index,continuous_audit=result['continuous_audit'])
        summary['scan_schedule_valid']=all(row['frame']==max(s['frame'] for s in result['scans']
            if round(s['delivered_s']*1e9)<=round(row['t_s']*1e9)) for row in result['calls'])
        if args.moving_dropout_only:
            ref=ROOT/'artifacts/validation/2026-10-26/m5_start_constraints_v1/run55_neighbor_departure/data/grid5_ego2_ahead_or_other_lane/report.json'
            raw=ref.read_bytes();reference=json.loads(raw)
            faulty=[r for r in result['calls'] if r['fault']]
            first,last=faulty[0]['t_s'],faulty[-1]['t_s']
            fields=('t_s','frame','captured_input','request_mps','request_steering','safe_mps','safe_steering','metadata','truth_pose')
            prefix=[r for r in result['calls'] if r['t_s']<first]
            old=[r for r in reference['calls'] if r['t_s']<first]
            summary['prefix_equal']=len(prefix)==len(old) and all(
                json_value({k:a[k] for k in fields})==json_value({k:b[k] for k in fields}) for a,b in zip(prefix,old))
            summary['reference_sha256']=hashlib.sha256(raw).hexdigest()
            summary['reference_unchanged']=ref.read_bytes()==raw
            summary['fault_first_s'],summary['fault_last_s']=first,last
            summary['moving_before_fault']=bool(prefix and prefix[-1]['safe_mps']>0)
            summary['all_fault_commands_zero']=all(r['request_mps']==r['safe_mps']==0 for r in faulty)
            recovered=[r for r in result['calls'] if r['t_s']>last and not r['explicit_stop'] and r['safe_mps']>0]
            summary['recovered_motion_s']=recovered[0]['t_s'] if recovered else None
        name=f'grid5_ego{index}_{policy}'+('_missing' if fault else '')
        case_dir=out/name;case_dir.mkdir()
        (case_dir/'report.json').write_text(json.dumps(json_value(result),ensure_ascii=False,allow_nan=False),encoding='utf-8')
        cases.append(dict(name=name,**summary));print(json.dumps(cases[-1]),flush=True)
    unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items())
    integrity=unchanged and all(c['scan_schedule_valid'] and c['calls']==270 and c['continuous_audit']['passed'] and
        c['motion_input_failures']==0 and c['explicit_stop_checked'] and c['stopped_after_request'] and
        c['fault_positive_requests']==0 and c['fault_positive_safe']==0 for c in cases)
    if args.moving_dropout_only:
        integrity &= all(c['prefix_equal'] and c['reference_unchanged'] and c['moving_before_fault'] and
                         c['all_fault_commands_zero'] and c['recovered_motion_s'] is not None for c in cases)
    summary=dict(cases=cases,sources_unchanged=unchanged,integrity_pass=integrity,
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
        scope='scripted neighboring lane moves first; same-lane rear stays fixed; not joint autonomous or physical start')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if integrity and len(cases)==len(definitions) else 1


if __name__=='__main__':raise SystemExit(main())

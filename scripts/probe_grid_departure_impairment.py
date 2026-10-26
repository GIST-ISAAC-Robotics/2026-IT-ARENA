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
    parser.add_argument('--lead-pause',action='store_true',help='registered pause at1.8s, resume3.8s; nominal and50ms')
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
    modes=(('pause_nominal',0.,0.),('pause_delay50',0.,.05)) if args.lead_pause else (
        ('minus30',-.03,0.),('plus30',.03,0.),('delay50',0.,.05),('minus30_delay50',-.03,.05))
    for name,bias,delay in modes:
        pause=dict(peer_stop_s=1.8,peer_resume_s=3.8) if args.lead_pause else {}
        result=simulate(ego,[s for s in slots if s!=ego],segments,road,vehicle,sensor,rounded=True,
            straight_priority=True,range_bias_m=bias,additional_scan_delay_s=delay,**pause)
        result['continuous_audit']=audit(result,slots,walls,road,geometry,ego_index=5)
        summary=result['summary'];summary['continuous_audit']=result['continuous_audit']
        summary['scan_schedule_valid']=all(row['frame']==max(s['frame'] for s in result['scans']
            if round(s['delivered_s']*1e9)<=round(row['t_s']*1e9)) for row in result['calls'])
        summary['expected_calls']=sum(round((.6018+i*.02)*1e9)>=round((.6018+delay)*1e9) for i in range(270))
        if args.lead_pause:
            reference=ROOT/'artifacts/validation/2026-10-26/m5_start_constraints_v1'/(
                'run40_departure_impairment/data/grid2_ego5_delay50/report.json' if delay else
                'run31_grid_coverage/data/grid2_ego5/report.json')
            raw_reference=reference.read_bytes();before=json.loads(raw_reference)['calls']
            old=[r for r in before if r['t_s']<1.8];new=[r for r in result['calls'] if r['t_s']<1.8]
            fields=('t_s','frame','captured_input','request_mps','request_steering','safe_mps','safe_steering',
                    'reason','metadata','truth_pose','truth_travel_m')
            summary['pre_pause_calls']=len(new)
            summary['pre_pause_exact']=len(old)==len(new) and all(
                all(json_value(a[k])==json_value(b[k]) for k in fields) for a,b in zip(old,new))
            summary['reference_sha256']=hashlib.sha256(raw_reference).hexdigest()
            summary['reference_unchanged']=reference.read_bytes()==raw_reference
            active=[r for r in result['calls'] if 1.8<=r['t_s']<5.6]
            holds=[r for r in active if r['t_s']<3.8 and r['safe_mps']==0. and r['truth_travel_m']>0.]
            after=[r for r in active if r['t_s']>=3.8 and r['safe_mps']>0.]
            longest=0.;start=None
            for row in active:
                if row['t_s']>=3.8:break
                if row['safe_mps']==0. and row['truth_travel_m']>0.:
                    if start is None:start=row['t_s']
                    longest=max(longest,min(3.8,row['t_s']+.02)-start)
                else:start=None
            summary['pause_response']=dict(zero_calls_after_departure=len(holds),
                first_zero_s=holds[0]['t_s'] if holds else None,
                first_positive_after_peer_resume_s=after[0]['t_s'] if after else None,
                longest_zero_interval_during_pause_s=longest,
                observed_stop_then_restart=bool(holds and after),
                scope='ideal command/state evidence only; physical stable stop not modeled')
        case_dir=out/('grid2_ego5_'+name);case_dir.mkdir()
        (case_dir/'report.json').write_text(json.dumps(json_value(result),ensure_ascii=False,allow_nan=False),encoding='utf-8')
        cases.append(dict(name=name,**summary));print(json.dumps(cases[-1]),flush=True)
    unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items())
    integrity=unchanged and all(c['scan_schedule_valid'] and c['calls']==c['expected_calls'] and
        c['continuous_audit']['passed'] and c['motion_input_failures']==0 and c['explicit_stop_checked'] and
        c['stopped_after_request'] and c.get('pre_pause_exact',True) and c.get('reference_unchanged',True) for c in cases)
    summary=dict(cases=cases,sources_unchanged=unchanged,integrity_pass=integrity,
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
        nominal_reference='run31 grid2ego5; run40 delay50 for pre-pause prefix' if args.lead_pause else
                          'run31_grid_coverage/data/grid2_ego5/report.json; no nominal rerun',
        scope='scripted lead plus ideal ego, sensor impairments only; no MCU/ROS/physics/deadline proof')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if integrity and len(cases)==len(modes) else 1


if __name__=='__main__':raise SystemExit(main())

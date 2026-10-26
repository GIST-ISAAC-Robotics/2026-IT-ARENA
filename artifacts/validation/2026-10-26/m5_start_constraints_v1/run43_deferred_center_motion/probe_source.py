"""기본 OFF 저장 출력 대조 뒤 명시 ON의 새 이상 궤적을 비교한다. 기본 채택 없음."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import yaml
from shapely.geometry import MultiLineString
from arena_autonomy.follow_trace import freeze,replay_call
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import ExecutionConfig,M3ExecutionFollow
from probe_grid_departure_observation import motion_history
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
    base=ROOT/'artifacts/validation/2026-10-26/m5_start_constraints_v1'
    vehicle=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar'];wb=vehicle['drivetrain']['wheelbase_m']
    slots=registry(json.loads((RUNTIME/'scene.json').read_text(encoding='utf-8')),(RUNTIME/'world.sdf').read_bytes(),vehicle)
    segments=wall_sections((RUNTIME/'world.sdf').read_bytes(),sensor['xyz_m'][2])
    walls=MultiLineString(segments.tolist());road=road_polygon(RUNTIME/'world.sdf')
    geometry=VehicleGeometry(wheelbase_m=wb,track_width_m=vehicle['drivetrain']['track_width_m'])
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
    cfg=FollowConfig(wheelbase_m=wb,occupancy_timing_mode='per_ray_candidate',
        boundary_evidence_mode='corner_distance_candidate',corridor_fallback_enabled=True)
    ex=ExecutionConfig(command_fallback=True,release_to_baseline=True,small_center_straight=True,
        straight_surface_commitment=True,rounded_occupancy=True,straight_with_threat=True)
    motion=motion_history(wb,sensor).config
    references={}
    for name,ref,index in (
        ('pause',base/'run41_departure_pause/data/grid2_ego5_pause_nominal/report.json',None),
        ('pause_delay',base/'run41_departure_pause/data/grid2_ego5_pause_delay50/report.json',None),
        ('five',base/'run20_straight_dynamics/data/report.json',1)):
        raw=ref.read_bytes();saved=json.loads(raw);saved=saved if index is None else saved['cases'][index]
        controller=M3ExecutionFollow(cfg,ex,allow_simulation_trial=True);diff=[];count=0
        for i,call in enumerate(saved['calls']):
            if call['explicit_stop']:break
            actual=replay_call(controller,call['captured_input'],motion);count+=1
            if freeze(actual)!=[call['request_mps'],call['request_steering'],call['metadata']]:diff.append(i)
        references[name]=dict(path=ref.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(raw).hexdigest(),
            unchanged=ref.read_bytes()==raw,calls=count,output_differences=diff,summary=saved['summary'])
    off_ok=all(r['unchanged'] and not r['output_differences'] for r in references.values())
    (out/'off_replay.json').write_text(json.dumps(references,ensure_ascii=False,indent=2),encoding='utf-8')
    if not off_ok:return 1
    cases=[]
    for name,count,delay,pause,fault in (('pause',2,0.,True,False),('pause_delay',2,.05,True,False),
                                        ('five',5,0.,False,False),('five_missing',5,0.,False,True)):
        selected=slots[:count];ego=next(s for s in selected if s['index']==5)
        result=simulate(ego,[s for s in selected if s!=ego],segments,road,vehicle,sensor,rounded=True,
            straight_priority=True,defer_center_on_straight=True,additional_scan_delay_s=delay,fault=fault,
            **(dict(peer_stop_s=1.8,peer_resume_s=3.8) if pause else {}))
        result['continuous_audit']=audit(result,selected,walls,road,geometry,ego_index=5)
        summary=result['summary'];summary['continuous_audit']=result['continuous_audit']
        summary['scan_schedule_valid']=all(row['frame']==max(s['frame'] for s in result['scans']
            if round(s['delivered_s']*1e9)<=round(row['t_s']*1e9)) for row in result['calls'])
        summary['expected_calls']=sum(round((.6018+i*.02)*1e9)>=round((.6018+delay)*1e9) for i in range(270))
        summary['deferred_straight_calls']=sum(c['metadata'].get('m3_lateral',{}).get('deferred_center',{}).get('ok',False)
            and c['metadata'].get('m3_candidate')=='straight' for c in result['calls'])
        case_dir=out/(f'grid{count}_ego5_'+name);case_dir.mkdir()
        (case_dir/'report.json').write_text(json.dumps(json_value(result),ensure_ascii=False,allow_nan=False),encoding='utf-8')
        cases.append(dict(name=name,**summary));print(json.dumps(cases[-1]),flush=True)
    unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items())
    integrity=unchanged and off_ok and all(c['scan_schedule_valid'] and c['calls']==c['expected_calls'] and
        c['continuous_audit']['passed'] and c['motion_input_failures']==0 and c['explicit_stop_checked'] and
        c['stopped_after_request'] and c['fault_positive_requests']==0 and c['fault_positive_safe']==0 for c in cases)
    summary=dict(cases=cases,off_replay=references,sources_unchanged=unchanged,integrity_pass=integrity,
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
        scope='explicit ON ideal motion; default OFF, no ROS/Gazebo/physical braking or hardware proof')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if integrity and len(cases)==4 else 1


if __name__=='__main__':raise SystemExit(main())

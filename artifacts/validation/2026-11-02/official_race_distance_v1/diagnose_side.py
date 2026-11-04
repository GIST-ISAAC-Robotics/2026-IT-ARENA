from pathlib import Path
import json,sys
root=Path.cwd();p=root/'artifacts/validation/2026-11-02/official_race_distance_v1';sys.path.insert(0,str(p));import run
cfg,overlay=run.expected()
line,scene=run.evaluator.load_centerline();road=run.evaluator.road_polygon(run.official.WORLD/'world.sdf');obs=run.evaluator.obstacle_polygons(run.official.WORLD/'world.sdf')
actor=run.actor_specs(cfg,run.overlay_case(cfg['cases'][0],run.OVERLAY))[1];start=run.evaluator.actor_start_s(cfg,scene,0,dict(actor_start_center_gap_m=actor['start_center_gap_m']))
case={**cfg['cases'][0],'schedule':actor['schedule'],'actor_lateral_offset_m':actor['lateral_offset_m'],'route_kind':actor['route_kind']}
result=[]
for offset in (.32,-.32):
    candidate={**case,'actor_lateral_offset_m':offset}
    errors=[]
    for i in range(526):
        s=start+5.25*i/525
        poly=run.evaluator.ev.rectangle(*run.evaluator.actor_pose_at(line,s,[],0.,offset),actor['length_m'],actor['width_m'])
        hits=[j for j,o in enumerate(obs) if poly.intersection(o).area>1e-6]
        if hits or poly.intersection(road).area>1e-6:
            errors.append(dict(s=s,road_overlap_area=poly.intersection(road).area,obstacles=hits))
    try:
        r=run.evaluator.check_route({**cfg,'actor':actor},line,candidate,start,road,obs)
    except Exception as e:r=dict(error=str(e))
    result.append(dict(offset=offset,route=r,errors=errors[:10],error_count=len(errors)))
(p/'side_geometry_diagnosis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps(result,ensure_ascii=False)[:3000])

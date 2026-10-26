"""동결 질의 함수와 현재 함수의 교차 순서 비용·전체 출력 대조. 새 주행 없음."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile

import numpy as np
import yaml
from arena_autonomy.follow_trace import freeze, restore_call, motion_scene
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy import observed_lateral_trial as observed
from arena_autonomy import m3_observed_follow as old_m3
from arena_autonomy import m3_execution_trial as m3
from probe_grid_departure_observation import motion_history
from probe_grid_follow import ROOT,CAMERA,VEHICLE


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();out=args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):p.error('new validation directory required')
    out.mkdir(parents=True,exist_ok=False)
    base=ROOT/'artifacts/validation/2026-10-26/m5_start_constraints_v1'
    snapshot=base/'run33_cost_profile/data/sources.zip';snapshot_raw=snapshot.read_bytes()
    with zipfile.ZipFile(snapshot) as z:
        source=z.read('src/arena_autonomy/arena_autonomy/observed_lateral_trial.py')
    tree=ast.parse(source)
    node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='path_observation_queries')
    namespace=dict(observed.__dict__)
    exec(compile(ast.Module(body=[node],type_ignores=[]),'frozen_run33_queries','exec'),namespace)
    functions={'frozen':namespace['path_observation_queries'],'current':observed.path_observation_queries}
    modules=(observed,old_m3,m3);originals=[m.path_observation_queries for m in modules]
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    wb=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']['drivetrain']['wheelbase_m']
    cfg=FollowConfig(wheelbase_m=wb,occupancy_timing_mode='per_ray_candidate',
        boundary_evidence_mode='corner_distance_candidate',corridor_fallback_enabled=True)
    ex=m3.ExecutionConfig(command_fallback=True,release_to_baseline=True,small_center_straight=True,
        straight_surface_commitment=True,rounded_occupancy=True,straight_with_threat=True)
    motion=motion_history(wb,sensor).config
    sources={Path(__file__),CAMERA,VEHICLE}|set((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    for module in tuple(sys.modules.values()):
        f=getattr(module,'__file__',None)
        if f:
            q=Path(f).resolve()
            if q.is_relative_to(ROOT) and q.suffix=='.py':sources.add(q)
    frozen={q.relative_to(ROOT).as_posix():q.read_bytes() for q in sources}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for n,b in frozen.items():archive.writestr(n,b)
    results=[];all_rows={}
    try:
        for name in ('grid5_ego4','grid2_ego2','grid3_ego1'):
            path=base/'run31_grid_coverage/data'/name/'report.json';raw=path.read_bytes();report=json.loads(raw)
            controllers={key:m3.M3ExecutionFollow(cfg,ex,allow_simulation_trial=True) for key in functions}
            differences={key:[] for key in functions};rows=[]
            for i,call in enumerate(report['calls']):
                if call['explicit_stop']:break
                row=dict(call=i,t_s=call['t_s'],reason=call['reason'])
                keys=('frozen','current') if i%2==0 else ('current','frozen')
                for key in keys:
                    for module in modules:module.path_observation_queries=functions[key]
                    data,scan,history=restore_call(call['captured_input'],motion)
                    scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                        epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
                    began=time.perf_counter_ns()
                    actual=controllers[key].command(scene,history,data['speed_mps'],data['side'],
                        data['previous_steering_rad'],data['dt_s'])
                    row[key+'_ms']=(time.perf_counter_ns()-began)/1e6
                    if freeze(actual)!=[call['request_mps'],call['request_steering'],call['metadata']]:differences[key].append(i)
                rows.append(row)
            metrics={key:dict(median_ms=float(np.median([r[key+'_ms'] for r in rows])),
                p95_ms=float(np.percentile([r[key+'_ms'] for r in rows],95)),
                max_ms=max(r[key+'_ms'] for r in rows),over20_ms=sum(r[key+'_ms']>20 for r in rows)) for key in functions}
            results.append(dict(case=name,calls=len(rows),metrics=metrics,output_differences=differences,
                input_sha256=hashlib.sha256(raw).hexdigest(),input_unchanged=path.read_bytes()==raw))
            all_rows[name]=rows
            print(json.dumps(results[-1]),flush=True)
    finally:
        for module,function in zip(modules,originals):module.path_observation_queries=function
    summary=dict(cases=results,sources_unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items()),
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
        reference_snapshot_sha256=hashlib.sha256(snapshot_raw).hexdigest(),
        reference_snapshot_unchanged=snapshot.read_bytes()==snapshot_raw,
        scope='alternating paired PC fixed-input command timing; not Jetson deadline or new closed-loop driving')
    summary['integrity_pass']=summary['sources_unchanged'] and summary['reference_snapshot_unchanged'] and all(
        c['calls']==250 and c['input_unchanged'] and not any(c['output_differences'].values()) for c in results)
    (out/'report.json').write_text(json.dumps(all_rows,ensure_ascii=False),encoding='utf-8')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if summary['integrity_pass'] and len(results)==3 else 1


if __name__=='__main__':raise SystemExit(main())

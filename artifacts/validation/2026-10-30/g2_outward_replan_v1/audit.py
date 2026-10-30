"""완료된 v1 입력을 그대로 재생해 미계산 여유와 마지막 정지 원인을 분리한다."""
from collections import Counter
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

BASE=Path(__file__).resolve().parent; ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from candidate import OutwardReplanGuide
from arena_autonomy.bypass_entry_trial import (EntryConfig, entry_path, _compare_entries,
    current_outer_wall_padding, preview_point_gap, preview_clearance)
from arena_autonomy.follow_trace import restore_call, freeze, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from run_closed import comparable


def interference(scene,guide,row,previous):
    config=guide.config
    wall=row['wall']
    points=np.asarray(scene.occupied_endpoints_xy_m)
    padding,classification=current_outer_wall_padding(scene,config,guide.side,wall)
    rays=[r for r in scene.rays if r.status=='return']
    hits=[]
    for steering in sorted({previous,row['steering_rad']}):
        curvature=math.tan(steering)/config.wheelbase_m
        for distance in np.arange(0.,2.501,.04):
            yaw=curvature*distance
            rx=(distance if abs(curvature)<1e-5 else math.sin(yaw)/curvature)-config.wheelbase_m/2
            ry=0. if abs(curvature)<1e-5 else (1-math.cos(yaw))/curvature
            dx,dy=points[:,0]-rx,points[:,1]-ry
            x=math.cos(yaw)*dx+math.sin(yaw)*dy-config.wheelbase_m/2
            y=-math.sin(yaw)*dx+math.cos(yaw)*dy
            indices=np.flatnonzero((abs(x)<=config.length_m/2+padding)&(abs(y)<=config.width_m/2+padding))
            if len(indices):
                hits.append(dict(steering_rad=steering,first_sample_distance_m=float(distance),
                    clearance_m=max(0.,float(distance)-.04),count=len(indices),returns=[dict(
                    point=points[i].tolist(),padding_m=float(padding[i]),
                    age_s=scene.t_ref_s-rays[i].acquired_at_s,source_index=rays[i].source_index,
                    swept_local_xy=[float(x[i]),float(y[i])]) for i in indices[:8]]))
                break
    path=np.asarray(row['path_points'])
    return dict(first_interference=hits,classification=classification,
        measured_preview_gap_m=preview_point_gap(path,points,padding,config),
        measured_preview_progress_m=preview_clearance(path,points,padding,config))


def main():
    trace=BASE/'closed1/one_outward_replan_026/input_output.jsonl'
    report=json.loads((BASE/'closed1/report.json').read_text())
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in report['source_sha256'].items())
    before=hashlib.sha256(trace.read_bytes()).hexdigest()
    counts=Counter(); samples=[]; evaluations=[]; seq=-1; footer=False
    for line in trace.open(encoding='utf-8'):
        event=json.loads(line);d=event['data']
        assert not footer and event['seq']==seq+1;seq=event['seq']
        if event['kind']=='header':
            assert d['schema']==908
            config=FollowConfig(**d['config']);motion=MotionConfig(**d['motion'])
            kwargs=dict(d['entry']);kwargs['offsets_m']=tuple(kwargs['offsets_m']);kwargs['transitions_m']=tuple(kwargs['transitions_m'])
            guide=OutwardReplanGuide(config,EntryConfig(**kwargs),**d['guide_kwargs'])
        elif event['kind']=='call':
            inp,scan,history=restore_call(d['input'],motion)
            scene=motion_scene(scan,history,inp['reference_s'],timing_verified=inp['timing_verified'],
                epoch=inp['epoch'],rear_half_angle_deg=inp['rear_half_angle_deg'])
            result=guide.evaluate(scene,history,inp['speed_mps'],inp['previous_steering_rad'],inp['dt_s'])
            assert equal_output(comparable(freeze(result)),comparable(d['output']))
            counts['matched']+=1
            if scene.t_ref_s>=5. and scene.t_ref_s<10.5:
                counts['late_calls']+=1
                counts['late_selected']+=result['selected'] is not None
                counts['late_safe_positive']+=d['safe'][0]>0
                if result['candidates']:
                    row=result['candidates'][0];counts['late_'+row['reason']]+=1
            if result.get('outward_replan',{}).get('applied'):
                reference=result['reference'];wall=reference['walls'][guide.side]
                fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
                for transition in guide.entry.transitions_m:
                    path=entry_path(fit,guide.offset,config,guide.entry,transition)
                    true=_compare_entries(scene,scene.t_ref_s,inp['speed_mps'],inp['previous_steering_rad'],inp['dt_s'],
                        config=config,entry=guide.entry,stationary_boundaries=True,
                        guide=dict(reference=reference,path=path,offset_m=guide.offset,transition_m=transition,
                            outer_side=guide.side,stationary_outer_boundary=True))
                    r=true['candidates'][0]
                    evaluations.append(dict(reference_s=scene.t_ref_s,transition_m=transition,
                        stored_preview_gap_m=0.,computed_preview_gap_m=r['preview_point_gap_m'],
                        computed_preview_progress_m=r['preview_return_progress_m'],
                        command_clearance_m=r['command_clearance_m'],eligible=r['eligible'],
                        reason=r['reason']))
            if any(abs(scene.t_ref_s-t)<1e-8 for t in (3.2,3.28,4.9,5.,5.08,9.5,9.58,10.48)) and result['candidates']:
                row=result['candidates'][0]
                geometry=interference(scene,guide,{**row,'wall':result['reference']['walls'][guide.side]},inp['previous_steering_rad'])
                samples.append(dict(reference_s=scene.t_ref_s,source_first_s=scene.source_first_s,
                    measured_speed_mps=inp['speed_mps'],command=d['command'],safe=d['safe'],
                    safety_reason=d['safety_reason'],reason=result['reason'],candidate_reason=row['reason'],
                    command_clearance_m=row['command_clearance_m'],motion_region=row['motion_region'],
                    offset_m=guide.offset,transition_m=guide.transition,geometry=geometry))
        elif event['kind']=='footer':
            footer=True;assert d['calls']==counts['matched']
        else:raise ValueError('unexpected record')
    assert footer and hashlib.sha256(trace.read_bytes()).hexdigest()==before
    output=BASE/'audit1.json'
    with output.open('x',encoding='utf-8') as f:
        json.dump(freeze(dict(counts=dict(counts),input_sha256=before,input_unchanged=True,
            source_matched=True,replan_gap_audit=evaluations,samples=samples,
            scope='recorded sensor replay; no new actuation; no truth inputs')),f,indent=2,allow_nan=False)
        f.write('\n')
    print(json.dumps(dict(counts=dict(counts),replan_gap_audit=evaluations,
        stops=[dict(t=r['reference_s'],reason=r['candidate_reason'],clearance=r['command_clearance_m'],
            safe=r['safe'],guard=r['safety_reason']) for r in samples]),indent=2))


if __name__=='__main__':main()

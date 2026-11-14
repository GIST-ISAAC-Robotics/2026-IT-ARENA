"""저장 native LiDAR의 평가 전용 actor 연관 근사. ROS/제어기 import·발행 없음."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import statistics
import xml.etree.ElementTree as ET

import numpy as np


def read_json(path):
    return json.loads(path.read_bytes())


def file_sha(path):
    result=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):result.update(block)
    return result.hexdigest()


def pose_value(element):
    pose=element.find('pose')
    if pose is not None and pose.get('relative_to'):
        raise ValueError('relative_to pose requires an explicit frame resolver')
    value=np.array([float(x) for x in (pose.text if pose is not None else '0 0 0 0 0 0').split()])
    if value.shape!=(6,) or not np.isfinite(value).all() or np.any(np.abs(value[3:5])>1e-9):
        raise ValueError('this audit supports nominal planar poses only')
    return value


def combine_pose(parent,child):
    c,s=math.cos(parent[5]),math.sin(parent[5])
    return np.array([parent[0]+c*child[0]-s*child[1],parent[1]+s*child[0]+c*child[1],
        parent[2]+child[2],0.,0.,parent[5]+child[5]])


def geometry(run):
    vehicle=ET.parse(run/'runtime_vehicle.sdf').getroot()
    matches=[(link,sensor) for link in vehicle.findall('.//link') for sensor in link.findall('sensor')
        if sensor.get('type')=='gpu_lidar']
    if len(matches)!=1:raise ValueError('exactly one recorded GPU lidar required')
    link,sensor=matches[0]
    mount=combine_pose(pose_value(link),pose_value(sensor))
    low=float(sensor.findtext('lidar/range/min'));high=float(sensor.findtext('lidar/range/max'))
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as stream:header=json.loads(next(stream))
    motion=header['data']['motion_config']
    expected=np.array([motion['lidar_x_m'],motion['lidar_y_m'],motion['lidar_yaw_rad']])
    if not np.allclose(mount[[0,1,5]],expected,atol=1e-9,rtol=0):
        raise ValueError('runtime SDF and recorded motion extrinsic disagree')
    actor_root=ET.parse(run/'evaluation_actor.sdf').getroot()
    models=actor_root.findall(".//model[@name='eval_g1b_actor']")
    if len(models)!=1:raise ValueError('exactly one primary actor model required')
    model=models[0];body=model.find("link[@name='body']")
    actor_z=pose_value(model)[2]
    body_pose=pose_value(body)
    collision=body.find('collision');collision_pose=combine_pose(body_pose,pose_value(collision))
    size=np.array([float(x) for x in collision.findtext('geometry/box/size').split()])
    if size.shape!=(3,) or np.any(size<=0):raise ValueError('positive actor collision box required')
    boxes=[]
    for visual in body.findall('visual'):
        box=visual.find('geometry/box/size')
        if box is None:continue
        position=combine_pose(body_pose,pose_value(visual))
        dimensions=np.array([float(x) for x in box.text.split()])
        z=actor_z+position[2]
        if z-dimensions[2]/2 <= mount[2] <= z+dimensions[2]/2:
            boxes.append(dict(name=visual.get('name'),xy=position[:2].tolist(),yaw=float(position[5]),
                size_xy=dimensions[:2].tolist(),z_interval_m=[float(z-dimensions[2]/2),float(z+dimensions[2]/2)]))
    return dict(lidar_xyz_m=mount[:3].tolist(),lidar_yaw_rad=float(mount[5]),
        source_frame_id=motion['lidar_frame_id'],range_min_m=low,range_max_m=high,
        actor_box_xy=collision_pose[:2].tolist(),actor_box_yaw_rad=float(collision_pose[5]),
        actor_box_size_xy_m=size[:2].tolist(),actor_model_z_m=float(actor_z),active_visual_boxes=boxes,
        sparse_post_count=sum(b['name'].startswith('visual_post_') for b in boxes),
        height_basis='nominal recorded SDF; planar pose logs omit dynamic z/pitch/roll',
        extrinsic_scope='actual selected runtime SDF crosschecked with recorded motion config; not new physical calibration')


def load_poses(path,max_rows=None):
    rows=[]
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            rows.append(json.loads(line)[:4])
            if max_rows is not None and len(rows)>=max_rows:break
    poses=np.asarray(rows,dtype=float)
    if poses.ndim!=2 or poses.shape[1]!=4 or len(poses)<2 or not np.isfinite(poses).all():
        raise ValueError('finite [time,x,y,yaw] pose rows required')
    if np.any(np.diff(poses[:,0])<=0):raise ValueError('pose stamps must increase strictly')
    poses[:,3]=np.unwrap(poses[:,3])
    return poses


def interpolate(poses,times,max_gap):
    index=np.searchsorted(poses[:,0],times,side='right')-1
    index=np.clip(index,0,len(poses)-2)
    gap=poses[index+1,0]-poses[index,0]
    supported=(times>=poses[0,0])&(times<=poses[-1,0])&(gap<=max_gap)
    fraction=(times-poses[index,0])/gap
    values=poses[index,1:]+fraction[:,None]*(poses[index+1,1:]-poses[index,1:])
    return values,supported


def actor_frame(ranges,angles,ego,actor,mount):
    angle=angles+mount['lidar_yaw_rad']
    x=mount['lidar_xyz_m'][0]+ranges*np.cos(angle)
    y=mount['lidar_xyz_m'][1]+ranges*np.sin(angle)
    wx=ego[:,0]+np.cos(ego[:,2])*x-np.sin(ego[:,2])*y-actor[:,0]
    wy=ego[:,1]+np.sin(ego[:,2])*x+np.cos(ego[:,2])*y-actor[:,1]
    return np.column_stack((np.cos(actor[:,2])*wx+np.sin(actor[:,2])*wy,
                           -np.sin(actor[:,2])*wx+np.cos(actor[:,2])*wy))


def box_distance(points,xy,size,yaw=0.):
    delta=points-np.asarray(xy)
    c,s=math.cos(yaw),math.sin(yaw)
    local=np.column_stack((c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1]))
    outside=np.maximum(0.,np.abs(local)-np.asarray(size)/2)
    return np.hypot(outside[:,0],outside[:,1])


def numeric(value):
    return type(value) in (int,float) and math.isfinite(value)


def decode(row):
    if row.get('representation')!='native_measurements_v1' or row.get('time_basis')!='simulation_acquisition':
        raise ValueError('requires saved native arrays with simulation_acquisition stamps')
    ranges,angles,times=(row.get(k) for k in ('ranges','angles_rad','acquired_at_s'))
    if not all(isinstance(v,list) for v in (ranges,angles,times)) or not len(ranges)==len(angles)==len(times) or not ranges:
        raise ValueError('native array lengths disagree')
    if not all(numeric(v) for v in [*angles,*times]) or not numeric(row.get('stamp_s')):
        raise ValueError('finite ray angles/times required')
    times=np.asarray(times,dtype=float)
    if np.any(np.diff(times)<=0) or abs(times[0]-row['stamp_s'])>1e-8:raise ValueError('invalid ray timing')
    if type(row.get('source_epoch')) is not int or row['source_epoch']<0:raise ValueError('invalid source epoch')
    if not all(numeric(v) or v is None or v in ('+inf','-inf') for v in ranges):raise ValueError('invalid raw range type')
    return np.array([float(v) if numeric(v) else np.nan for v in ranges]),np.asarray(angles),times


def scan_audit(row,ego,actor,mount,window,epsilon,max_pose_gap):
    ranges,angles,times=decode(row)
    if row.get('source_frame_id')!=mount['source_frame_id']:raise ValueError('source frame/extrinsic mismatch')
    selected=(times>=window[0])&(times<=window[1])
    if not np.any(selected):return None,[]
    original_indices=np.flatnonzero(selected)
    ranges,angles,times=ranges[selected],angles[selected],times[selected]
    ep,eok=interpolate(ego,times,max_pose_gap);ap,aok=interpolate(actor,times,max_pose_gap)
    supported=eok&aok
    finite=np.isfinite(ranges)&(ranges>=mount['range_min_m'])&(ranges<mount['range_max_m'])
    keep=supported&finite
    points=actor_frame(ranges[keep],angles[keep],ep[keep],ap[keep],mount)
    distance=box_distance(points,mount['actor_box_xy'],mount['actor_box_size_xy_m'],mount['actor_box_yaw_rad'])
    associated=distance<=epsilon
    visual_distance=np.full(len(points),np.inf)
    for box in mount['active_visual_boxes']:
        visual_distance=np.minimum(visual_distance,box_distance(points,box['xy'],box['size_xy'],box['yaw']))
    near_visual=associated&(visual_distance<=epsilon)
    complete=bool(np.all(supported))
    indices=original_indices[keep]
    samples=[dict(ray=int(indices[i]),acquired_at_s=float(times[keep][i]),range_m=float(ranges[keep][i]),
        actor_xy_m=points[i].tolist(),box_outside_distance_m=float(distance[i]),
        near_nominal_visual=bool(near_visual[i])) for i in np.flatnonzero(associated)]
    result=dict(scan_stamp_s=row['stamp_s'],source_epoch=row['source_epoch'],
        first_ray_s=float(times[0]),last_ray_s=float(times[-1]),selected_raw_rays=len(times),
        finite_in_range_returns=int(np.sum(finite)),pose_unsupported_rays=int(np.sum(~supported)),
        evaluated_returns=int(np.sum(keep)),association_complete=complete,
        estimated_actor_box_associated_returns=int(np.sum(associated)) if complete else None,
        partial_supported_box_returns=int(np.sum(associated)),
        near_nominal_visual_returns=int(np.sum(near_visual)) if complete else None,
        box_only_ambiguous_returns=int(np.sum(associated&~near_visual)) if complete else None,
        associated_range_m=[float(v) for v in ranges[keep][associated]],examples=samples[:4])
    return result,samples


def stats(values):
    return None if not values else dict(count=len(values),min=min(values),median=statistics.median(values),max=max(values))


def zero_spans(rows):
    starts=[r['first_ray_s'] for r in rows]
    differences=[b-a for a,b in zip(starts,starts[1:]) if b>a]
    gap_limit=1.5*statistics.median(differences) if differences else None
    spans=[];active=None;previous=None;missing=[]
    for row in rows:
        broken=(previous is not None and (row['source_epoch']!=previous['source_epoch'] or
            gap_limit is not None and row['first_ray_s']-previous['first_ray_s']>gap_limit))
        if broken:missing.append([previous['last_ray_s'],row['first_ray_s']])
        zero=row['association_complete'] and row['estimated_actor_box_associated_returns']==0
        if broken or not zero:
            if active is not None:spans.append(active)
            active=None
        if zero:
            if active is None:active=dict(start_s=row['first_ray_s'],end_s=row['last_ray_s'],scans=0)
            active['end_s']=row['last_ray_s'];active['scans']+=1
        previous=row
    if active is not None:spans.append(active)
    for row in spans:row['duration_s']=max(0.,row['end_s']-row['start_s'])
    return dict(longest_confirmed_zero_association_scan_span_s=max((r['duration_s'] for r in spans),default=0.),
        zero_spans=spans,unobserved_scan_gaps=missing,scan_start_gap_limit_s=gap_limit,
        meaning='completed-scan association absence; not proof of expected actor visibility or detector failure')


def audit(run,output_dir,epsilon=.015,max_pose_gap=.05):
    if epsilon<0 or not math.isfinite(epsilon) or max_pose_gap<=0 or not math.isfinite(max_pose_gap):raise ValueError('invalid tolerances')
    report=read_json(run/'report.json')
    window=[report['case_window']['green_sim_s'],report['case_window']['end_sim_s']]
    if not all(numeric(v) for v in window) or window[1]<=window[0]:raise ValueError('completed positive case window required')
    mount=geometry(run)
    ego,actor=load_poses(run/'ego_poses.jsonl'),load_poses(run/'actor_poses.jsonl')
    rows=[];seen={};duplicates=0;all_ranges=[];examples=[]
    with (run/'scans_received.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row=json.loads(line)
            key=(row.get('source_epoch'),row.get('stamp_s'))
            digest=hashlib.sha256(line.strip().encode()).hexdigest()
            if key in seen:
                if seen[key]!=digest:raise ValueError('same scan key with different payload')
                duplicates+=1;continue
            seen[key]=digest
            result,samples=scan_audit(row,ego,actor,mount,window,epsilon,max_pose_gap)
            if result is None:continue
            if rows and result['first_ray_s']<=rows[-1]['first_ray_s']:raise ValueError('case-window scan order/reset unsupported')
            if result['association_complete']:all_ranges+=result['associated_range_m']
            if len(examples)<20:examples+=samples[:20-len(examples)]
            rows.append(result)
    complete=[r for r in rows if r['association_complete']]
    files=['report.json','runtime_vehicle.sdf','evaluation_actor.sdf','runtime_local_sensors.yaml',
        'scans_received.jsonl','ego_poses.jsonl','actor_poses.jsonl','follow_decisions.jsonl']
    result=dict(schema='sparse_actor_association_audit_v1',scope='offline evaluation only; no controller inputs',
        attribution='estimated actor-associated returns; not renderer object IDs or confirmed detections',
        run=str(run.resolve()),case_window_s=window,geometry=mount,association_tolerance_m=epsilon,max_pose_gap_s=max_pose_gap,
        selected_scans=len(rows),pose_supported_scans=len(complete),unsupported_scans=len(rows)-len(complete),duplicates_skipped=duplicates,
        raw_finite_returns=sum(r['finite_in_range_returns'] for r in rows),
        estimated_box_returns_supported_scans=sum(r['estimated_actor_box_associated_returns'] for r in complete),
        nominal_visual_near_returns_supported_scans=sum(r['near_nominal_visual_returns'] for r in complete),
        box_only_ambiguous_returns_supported_scans=sum(r['box_only_ambiguous_returns'] for r in complete),
        estimated_box_count_per_supported_scan=stats([r['estimated_actor_box_associated_returns'] for r in complete]),
        estimated_associated_range_m=stats(all_ranges),absence=zero_spans(rows),examples=examples,
        limitations=['nearby wall/occluder returns can enter either tolerance region; no unique actor attribution',
            'visual subset uses nominal SDF height; ego/actor pose logs omit dynamic z/pitch/roll',
            'pose interpolation and renderer sampling differ; tolerance is an evaluation assumption, not sensor accuracy',
            'box is the conservative collision envelope, not occupied sparse upper area',
            'missing pose/scan coverage is not zero returns; no pose extrapolation is included in counts'],
        input_sha256={name:file_sha(run/name) for name in files},original_report_passed=report.get('passed'),
        original_report_unchanged=True)
    output_dir.mkdir(parents=True,exist_ok=False)
    with (output_dir/'per_scan.jsonl').open('x',encoding='utf-8') as stream:
        for row in rows:stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    with (output_dir/'summary.json').open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--association-tolerance-m',type=float,default=.015)
    parser.add_argument('--max-pose-gap-s',type=float,default=.05)
    args=parser.parse_args()
    result=audit(args.run,args.output_dir,args.association_tolerance_m,args.max_pose_gap_s)
    print(json.dumps({k:result[k] for k in ['selected_scans','pose_supported_scans','raw_finite_returns',
        'estimated_box_returns_supported_scans','nominal_visual_near_returns_supported_scans','estimated_associated_range_m']},ensure_ascii=False))

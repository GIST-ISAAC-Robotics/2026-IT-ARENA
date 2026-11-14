"""Saved raw scans with reconstructed 50 Hz callbacks, not closed-loop replay."""
from pathlib import Path
import argparse, copy, json, sys
from collections import Counter
A=Path(__file__).resolve().parent;R=A.parents[3]
parser=argparse.ArgumentParser();parser.add_argument('--stage');parser.add_argument('--name',required=True)
args=parser.parse_args()
if args.stage:sys.path.insert(0,str(Path(args.stage)/'src/arena_autonomy'))
import numpy as np
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from arena_autonomy.scan_input import project_scan
from arena_autonomy.local_path import mask_scan,swept_limit,braking_speed
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from arena_autonomy.known_structure import structure_policy
run=R/'artifacts/validation/2026-11-14/field_wall_drive_r520_v1/run1'
rows=[json.loads(l) for l in (run/'follow_decisions.jsonl').read_text(encoding='utf-8').splitlines()]
header=next(r['data'] for r in rows if r['kind']=='header')
samples=[r['data']['input'] for r in rows if r['kind']=='call'];sample=samples[0]
scans=[json.loads(l) for l in (run/'scans_received.jsonl').read_text(encoding='utf-8').splitlines()]
cfg=header['follow_config']
policy=structure_policy(RecoveryPolicy,cfg['known_structure_sectors_json'])(allow_narrow_forward=True,
    surface_translation_enabled=True,lateral_metric_enabled=True,oblique_metric_enabled=True,crop_edge_component_trial=True)
policy.command_observation_priority=True
motion=MotionConfig(**header['motion_config']);history=MotionHistory(motion)
merged={key:{} for key in ('wheels','gyro')}
for item in samples:
    for key in merged:
        for stamp,value in zip(item['history'][key]['times'],item['history'][key]['values']):
            if stamp in merged[key] and merged[key][stamp]!=value:raise ValueError('conflicting saved sensor sample')
            merged[key][stamp]=value
ordered={key:sorted(values.items()) for key,values in merged.items()}
out=[];previous_angle=0.
for tick in range(151):
    now=6.007+tick*.02
    available=[raw for raw in scans if raw['acquired_at_s'][-1]<=now]
    if not available:continue
    raw=available[-1];data=copy.deepcopy(sample)
    data['scan']['measurements'].update(ranges_m=[float(v) for v in raw['ranges']],angles_rad=raw['angles_rad'],
        acquired_at_s=raw['acquired_at_s'],source_epoch=raw['source_epoch'],source_frame_id=raw['source_frame_id'],time_basis=raw['time_basis'])
    data['reference_s']=now
    for key in ordered:
        values=[(stamp,value) for stamp,value in ordered[key] if stamp<=now]
        series=getattr(history,key)
        series.times[:]=[p[0] for p in values];series.values[:]=[p[1] for p in values]
    _,scan,_=restore_call(data,motion)
    scan=mask_scan(scan,sample['rear_half_angle_deg'])
    points,ids,_=project_scan(scan,history,now,mode='both',timing_verified=True)
    measured=history.wheels.values[-1];requested=.5 if now>=7.72 else 0.;angle=.3 if requested else 0.
    partial=policy.evaluate(scan,points,history,now,max(requested,abs(measured)),angle,previous_angle,
        cfg['wheelbase_m'],cfg['length_m'],cfg['width_m'])
    crawl=None
    if args.stage and not partial['allowed'] and partial['reason']=='known_obstacle_latched':
        from arena_autonomy.guard_memory_command import limited_memory_command
        crawl=limited_memory_command(policy,partial,scan,points,history,now,requested,measured,
            angle,previous_angle,cfg['wheelbase_m'],cfg['length_m'],cfg['width_m'])
    allowed=partial['allowed'] or bool(crawl and crawl['accepted']);cap=partial.get('speed_cap_mps')
    if crawl and crawl['accepted']:cap=crawl['speed_cap_mps'] if cap is None else min(cap,crawl['speed_cap_mps'])
    clearance=swept_limit(points,angle,cfg['wheelbase_m'],length=cfg['length_m'],width=cfg['width_m'])
    safe=min(requested,braking_speed(clearance,now-scan.acquired_at_s[0]),cap if cap is not None else requested) if allowed else 0.
    memory=getattr(policy,'guard_memory',None)
    out.append(freeze(dict(now=now,source=scan.acquired_at_s[0],reason=partial['reason'],requested=requested,
        safe=safe,crawl=crawl,active=len(memory.points) if memory else None,raw_points=len(points),raw_clearance=clearance)))
    previous_angle=angle
dest=A/args.name;dest.mkdir(exist_ok=True)
(dest/'rows.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
positive=[row for row in out if row['requested']>0]
summary=dict(calls=len(out),positive_requests=len(positive),positive_outputs=sum(row['safe']>0 for row in positive),
    output_speeds=dict(Counter(row['safe'] for row in positive)),reasons=dict(Counter(row['reason'] for row in out)),
    candidate_reasons=dict(Counter((row['crawl'] or {}).get('reason') for row in positive)),
    maximum_active_points=max((row['active'] or 0) for row in out),
    scope='saved raw/acquisition histories; reconstructed callbacks, hypothetical command and recorded fixed ego motion; not closed loop or exact receipt timing')
(dest/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False))

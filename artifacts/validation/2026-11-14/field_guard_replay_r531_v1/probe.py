from pathlib import Path
import json,copy,math,sys
import numpy as np
A=Path(__file__).resolve().parent;R=A.parents[3]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scan_input import project_scan
from arena_autonomy.local_path import mask_scan
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from arena_autonomy.known_structure import structure_policy
P=R/'artifacts/validation/2026-11-14/field_wall_drive_r520_v1/failure_inputs'
rows=[json.loads(l) for l in (P/'controller_prefix.jsonl').read_text().splitlines()]
header=rows[0]['data'];sample=next(r['data']['input'] for r in rows if r['kind']=='call')
scans=[json.loads(l) for l in (P/'raw_scan_prefix.jsonl').read_text().splitlines()]
cfg=header['follow_config'];policy=structure_policy(RecoveryPolicy,cfg['known_structure_sectors_json'])(allow_narrow_forward=True,surface_translation_enabled=True,lateral_metric_enabled=True,oblique_metric_enabled=True,crop_edge_component_trial=True)
policy.command_observation_priority=True
motion=MotionConfig(**header['motion_config']);out=[]
for now in np.arange(6.007,7.727,.02):
    available=[s for s in scans if s['acquired_at_s'][-1]<=now]
    if not available:continue
    raw=available[-1];data=copy.deepcopy(sample)
    data['scan']['measurements'].update(ranges_m=[float(v) for v in raw['ranges']],angles_rad=raw['angles_rad'],acquired_at_s=raw['acquired_at_s'],source_epoch=raw['source_epoch'],source_frame_id=raw['source_frame_id'],time_basis=raw['time_basis'])
    data['reference_s']=float(now)
    for key in ('wheels','gyro'):
        old=data['history'][key];n=sum(t<=now for t in old['times']);old['times']=old['times'][:n];old['values']=old['values'][:n]
    data,scan,history=restore_call(data,motion)
    scan=mask_scan(scan,sample['rear_half_angle_deg'])
    points,ids,meta=project_scan(scan,history,float(now),mode='both',timing_verified=True)
    measured=history.wheels.values[-1]
    answer=policy.evaluate(scan,points,history,float(now),abs(measured),0.,0.,cfg['wheelbase_m'],cfg['length_m'],cfg['width_m'])
    evidence=getattr(policy,'latch_evidence',None)
    rec=dict(now=float(now),source=scan.acquired_at_s[0],reason=answer.get('reason'),allowed=answer.get('allowed'),answer=freeze(answer),evidence=freeze(evidence))
    if not out or rec['reason']!=out[-1]['reason']:print('change',round(now,3),rec['reason'], 'evidence',str(rec['evidence'])[:900])
    out.append(rec)
(A/'baseline.json').write_text(json.dumps(out,ensure_ascii=False,indent=2))
print('calls',len(out),'latched',sum(r['reason']=='known_obstacle_latched' for r in out))

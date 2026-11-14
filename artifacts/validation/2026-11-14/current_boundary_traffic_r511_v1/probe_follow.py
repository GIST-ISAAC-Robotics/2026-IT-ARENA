"""동일 저장 입력에서 앵커 차이만 비교한다. .25m는 원인 분리값이며 채택 후보가 아니다."""
import inspect_geometry as geometry
from dataclasses import replace
from collections import Counter
import json
from arena_autonomy.local_follow import LocalFollow
from arena_autonomy.race_runtime_policy import terminal_plan,released_plan

root,A=geometry.ROOT,geometry.A
source=A.parent/'moving_pass_facility_drive_r503_v1/run1/follow_decisions.jsonl'
plans=[]
for line in source.open(encoding='utf-8'):
    row=json.loads(line);kind,data=row['kind'],row['data']
    if kind=='runtime_evaluator_header':header=data['header']
    if kind=='runtime_plan_submitted' and 84<=data['sequence']<=120:plans.append(data)
cfg=geometry.FollowConfig(**header['follow_config'])
followers={'baseline':LocalFollow(cfg),'anchor_only_025_diagnostic':LocalFollow(replace(cfg,boundary_anchor_error_m=.25))}
expected={r['key'][1]:r['alternative'] for r in json.loads((A.parent/'current_follow_recovery_r510_v1/diagnostic.json').read_text())['comparisons']}
results=[]
terminal_plan.install()
try:
    with released_plan.enabled():
        for row in plans:
            data,scan,history=geometry.restore_call(row['input'],geometry.MotionConfig(**header['motion_config']))
            scene=geometry.motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
            output={}
            for name,follow in followers.items():
                result=follow.command(scene,history,abs(data['speed_mps']),data['side'],data['previous_steering_rad'],data['dt_s'],raw_scan=scan)
                detail=result[2]
                output[name]=dict(speed=result[0],steering=result[1],reason=detail.get('reason'),
                    selected=detail.get('selector_choice'),selection=detail.get('boundary_selection'),
                    accepted=detail.get('fresh_boundary_accepted'),clearance=detail.get('swept_clearance_m'),
                    anchor=follow.config.boundary_anchor_error_m,
                    partial_reason=(detail.get('partial_observation') or {}).get('reason'))
            old=expected[row['sequence']]
            baseline=output['baseline']
            matched=(abs(baseline['speed']-old[0])<1e-9 and abs(baseline['steering']-old[1])<1e-9 and baseline['reason']==old[2].get('reason'))
            results.append(dict(seq=row['sequence'],baseline_control_matches_r510=matched,output=output))
finally:terminal_plan.uninstall()
report=dict(scope='current saved inputs only; diagnostic parameter .25m not adopted; no closed-loop proof',
            source_sha256=geometry.hashes['moving_pass_facility_drive_r503_v1'],rows=results,
            baseline_control_matches_r510=sum(r['baseline_control_matches_r510'] for r in results),count=len(results))
(A/'anchor_probe.json').write_text(json.dumps(geometry.freeze(report),indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
for r in results:
    if r['seq']>=92:print(json.dumps(r))

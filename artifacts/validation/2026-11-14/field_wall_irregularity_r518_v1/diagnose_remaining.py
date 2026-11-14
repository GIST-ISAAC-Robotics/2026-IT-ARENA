from pathlib import Path
import importlib.util,json,sys
import numpy as np
A=Path(__file__).resolve().parent
sys.path[:0]=[str(A/'source_snapshot/arena_autonomy'),str(A/'source_snapshot/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from types import ModuleType
candidate=ModuleType('diagnostic_candidate')
candidate.rows=[]
text=(A/'bypass_wall_fit.candidate.py').read_text(encoding='utf-8')
needle='        if np.percentile(residual,80)>config.measurement_padding_m or np.max(residual)>association:continue'
replacement="""        rows.append(dict(point_count=len(part),span_x_m=float(np.ptp(part[:,0])),heading_deg=math.degrees(math.atan(a)),residual_p80_m=float(np.percentile(residual,80)),residual_max_m=float(np.max(residual)),source_indices=ids[piece].tolist(),small_joins=small_joins))
"""+needle
assert needle in text
exec(compile(text.replace(needle,replacement),'diagnostic_candidate','exec'),candidate.__dict__)
source=A.parent/'continuous_pass_preview_drive_r513_v1/run1/follow_decisions.jsonl'
for line in source.open():
    r=json.loads(line)
    if r['kind']=='runtime_evaluator_header':header=r['data']['header'];break
cfg=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
report=json.loads((A/'candidate_report.json').read_text());results=[]
for seq in (90,99):
    variant='s15_bend30_repeated_gap10'
    selected=next(x for x in report['probes'] if x['key'][1]==seq and x['variant']==variant)
    data=json.loads((A/'candidate'/f'input_seq{seq}_{variant}.json').read_text())
    d,scan,history=restore_call(data,motion)
    scene=motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    candidate.rows=[]
    fit,detail=candidate.tracked_straight_wall(scene,cfg,'left',selected['fit_calls'][-1]['remembered_wall'])
    out=json.loads((A/'candidate'/f'output_seq{seq}_{variant}.json').read_text())
    result=dict(seq=seq,fit=fit,detail=detail,fit_components=candidate.rows,entry=out[2]['moving_pass']['entry'],amplitude_assumed_m=.03,wavelength_assumed_m=selected['wavelength_assumed_m'])
    results.append(result)
(A/'remaining_failures.json').write_text(json.dumps(freeze(results),indent=2),encoding='utf-8')
for r in results:print(json.dumps({**r,'fit_components':[{k:v for k,v in row.items() if k!='source_indices'} for row in r['fit_components']]}))

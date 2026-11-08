"""마지막 실제 스캔의 경로/회랑 근거만 대조. 제어 명령·폐루프 실행 없음."""
import json
from pathlib import Path
import numpy as np
from arena_autonomy.local_follow import FollowConfig, path_agrees, path_agrees_arc
from arena_autonomy.local_path import local_path
from arena_autonomy.corner_corridor import short_prefix_evidence, corridor_evidence
from arena_autonomy.corridor_roles import role_aware_evidence

here = Path(__file__).resolve().parent
pick = json.loads((here/'corner_after.clouds.json').read_text())[-1]
points = np.array(next(c['points'] for c in pick['calls'] if c['kind']=='local_path'))
with (here/'run1/follow_decisions.jsonl').open() as f:
    header=json.loads(next(f))['data']
cfg=FollowConfig(**header['follow_config'])
paths=[local_path(points, side, cfg.wall_offset_m) for side in ('left','right')]
def compact(obj):
    if isinstance(obj, np.ndarray): return dict(shape=obj.shape)
    if isinstance(obj, dict): return {k:compact(v) for k,v in obj.items() if k not in ('path_points','source_indices','source_point_indices')}
    if isinstance(obj, list): return [compact(x) for x in obj]
    if isinstance(obj, np.generic): return obj.item()
    return obj
result=dict(t=pick['t'], wall_info=[i for p,i in paths],
            agreement=path_agrees(paths[0][0],paths[1][0],cfg.boundary_agreement_m),
            arc_agreement=path_agrees_arc(paths[0][0],paths[1][0],cfg.boundary_agreement_m),
            original=corridor_evidence(points,'left',cfg),
            roles=role_aware_evidence(points,'left',cfg),
            short=short_prefix_evidence(points,'left',cfg), swapped_short=short_prefix_evidence(points,'right',cfg))
result=compact(result)
(here/'probe_final_geometry_with_swap.json').write_text(json.dumps(result,indent=2,default=str),encoding='utf8')
print(json.dumps(result,ensure_ascii=False,default=str))

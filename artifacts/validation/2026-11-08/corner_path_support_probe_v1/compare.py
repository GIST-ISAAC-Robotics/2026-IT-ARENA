from pathlib import Path
import json,hashlib
import numpy as np
from candidate import propose
from arena_autonomy.local_follow import FollowConfig
HERE=Path(__file__).resolve().parent
records=json.loads((HERE/'snapshots.json').read_bytes())['captures'];results=[]
for row in records:
    if row['carried'] is None:continue
    points=np.asarray(row['points']);before=points.copy()
    cfg=FollowConfig(**row['config']);old=np.asarray(row['carried'])
    path,detail=propose(points,old,row['old_wall'],cfg,row['t']-row['old_path_source'])
    assert np.array_equal(points,before)
    if path is not None or 34.51<row['t']<34.73:
        results.append(dict(t=row['t'],detail=detail,path=None if path is None else path.tolist()))
out=dict(scope='isolated current-face geometry only; all occupancy unchanged; not drive permission',
    source_sha256=hashlib.sha256((HERE/'candidate.py').read_bytes()).hexdigest(),rows=results)
(HERE/'candidate_result.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf8')
print(json.dumps([dict(t=r['t'],**{k:v for k,v in r['detail'].items() if not k.endswith('indices')}) for r in results],indent=2))

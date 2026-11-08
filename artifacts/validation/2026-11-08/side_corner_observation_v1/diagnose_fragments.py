"""No candidate changes: record each existing rejection branch on one saved frame."""
import ast,collections,json,math
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.piecewise_wall import _vertices
HERE=Path(__file__).resolve().parent
raw=(HERE/'fragment_candidate.py').read_text()
counts=collections.Counter();samples={}
def capture(line,values):
    counts[line]+=1
    if len(samples.get(line,[]))>=3:return
    d={k:values[k] for k in ('gap','bmin','amax','dist','ia','ib') if k in values}
    for k in ('an','a','b','line','joined','t','center'):
        v=values.get(k)
        if isinstance(v,np.ndarray):
            d[k]=dict(shape=list(v.shape),min=v.min(axis=0).tolist() if len(v) else None,max=v.max(axis=0).tolist() if len(v) else None)
    if 'joined' in values:
        v=values['joined'];d['vertices']=v[_vertices(v)].tolist()
    samples.setdefault(line,[]).append(d)
class Instrument(ast.NodeTransformer):
    def visit_Continue(self,node):return [ast.copy_location(ast.Expr(ast.Call(ast.Name('capture',ast.Load()),[ast.Constant(node.lineno),ast.Call(ast.Name('locals',ast.Load()),[],[])],[])),node),node]
tree=Instrument().visit(ast.parse(raw));ast.fix_missing_locations(tree)
ns={'capture':capture};exec(compile(tree,'fragment_candidate_diagnostic','exec'),ns)
src=HERE.parent/'side_corner_live_v1/right_parallel_first_curve/run1/follow_decisions.jsonl'
for line in src.open():
    e=json.loads(line)
    if e['kind']=='header':mc=MotionConfig(**e['data']['motion_config']);cfg=e['data']['follow_config']
    if e['kind']=='call' and abs(e['data']['input']['reference_s']-33.52)<.001:
        i,scan,h=restore_call(e['data']['input'],mc)
        s=motion_scene(scan,h,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
        pts=np.asarray(s.occupied_endpoints_xy_m)
        _,info=ns['propose'](pts,'right',cfg['wall_offset_m'],cfg['wheelbase_m']/math.tan(cfg['max_steering_rad']))
        result={'info':info,'branches':{str(k):{'count':v,'line':raw.splitlines()[k-1].strip(),'samples':samples.get(k,[])} for k,v in counts.items()}}
        (HERE/'fragment_rejections.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:{'count':v['count'],'line':v['line']} for k,v in result['branches'].items()}));break

"""r36 저장 판단 재현과 마지막 반복 정체의 점유 원호 분리. 실제 실행 소스는 변경하지 않는다."""
from pathlib import Path
import hashlib,importlib.util,json,math,sys,types,zipfile
import numpy as np
from arena_autonomy.follow_trace import MotionConfig,FollowConfig,replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
import arena_autonomy.bypass_entry_trial as entry

A=Path(__file__).resolve().parent;parent=A.parent/'entry_preview_wiring_v1'
trace=parent/'run1/follow_decisions.jsonl'
# r37의 다른 기능 수정 전(r36) 모듈을 해시 확인 후 이 분석 프로세스에서만 사용한다.
with zipfile.ZipFile(A.parent/'c1_query_side_scope_v1/before.zip') as archive:
    source=archive.read('src/arena_autonomy/arena_autonomy/same_path_gap_trial.py')
contract=json.loads((A.parents[3]/'config/tests/query_local_side_scope_r37_sha256.json').read_bytes())
assert hashlib.sha256(source).hexdigest()==contract['approved_changes']['src/arena_autonomy/arena_autonomy/same_path_gap_trial.py']['before']
legacy=types.ModuleType('arena_autonomy.same_path_gap_trial');exec(compile(source,'preserved_r36_same_path','exec'),legacy.__dict__)
sys.modules['arena_autonomy.same_path_gap_trial']=legacy
original=entry.swept_limit;now=None;captures=[];active=None

def instrument(points,steering,*args,**kw):
    result=original(points,steering,*args,**kw)
    if active is not None and kw.get('horizon')==2.5:
        pts=np.asarray(points);pad=np.broadcast_to(kw['margin'],(len(pts),));length=kw['length'];width=kw['width'];wheelbase=args[0]
        hits=[]
        for distance in (0.,.04):
            curvature=math.tan(steering)/wheelbase;yaw=curvature*distance;c=math.cos(yaw);s=math.sin(yaw)
            rx=distance-wheelbase/2 if abs(curvature)<1e-5 else s/curvature-wheelbase/2
            ry=0. if abs(curvature)<1e-5 else (1-c)/curvature
            dx,dy=pts[:,0]-rx,pts[:,1]-ry
            inside=(abs(c*dx+s*dy-wheelbase/2)<=length/2+pad)&(abs(-s*dx+c*dy)<=width/2+pad)
            for k in np.flatnonzero(inside):
                hits.append(dict(distance_m=distance,index=int(k),xy=pts[k].tolist(),padding_m=float(pad[k]),
                    current_unpadded_clearance_xy=[float(abs(pts[k,0])-length/2),float(abs(pts[k,1])-width/2)]))
        active['sweeps'].append(dict(steering_rad=steering,limit_m=result,near_hits=hits,
            points=pts.tolist(),padding=pad.tolist()))
    return result
entry.swept_limit=instrument
matched=0
for line in trace.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        header=d;motion=MotionConfig(**d['motion_config'])
        engine=BypassHandoffFollow(FollowConfig(**d['follow_config']),HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
    elif e['kind']=='reset':engine.reset()
    elif e['kind']=='call':
        now=d['input']['reference_s']
        if now>35.561:break
        active=dict(t=now,input=d['input'],output=d['output'],sweeps=[]) if 35.519<=now<=35.561 else None
        actual=freeze(replay_call(engine,d['input'],motion));matched+=1
        assert equal_output(comparable_decision(actual),comparable_decision(d['output'])),now
        if active is not None:captures.append(active)
assert len(captures)==3
(A/'snapshots.json').write_text(json.dumps(freeze(dict(header=header,captures=captures)),ensure_ascii=False)+'\n',encoding='utf8')
summary=[]
for cap in captures:
    unique={}
    for sweep in cap['sweeps']:
        for hit in sweep['near_hits']:
            unique[(hit['index'],hit['distance_m'])]=hit
    summary.append(dict(t=cap['t'],measured_mps=cap['input']['speed_mps'],previous_rad=cap['input']['previous_steering_rad'],
        output=cap['output'][:2],sweeps=len(cap['sweeps']),near_hits=list(unique.values())))
out=dict(scope='same saved r36 inputs, exact prefix; no simulation or changed command',matched=matched,
    trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),summary=summary)
(A/'result.json').write_text(json.dumps(freeze(out),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print('r36 matched',matched,'captured',[(r['t'],len(r['near_hits'])) for r in summary],flush=True)

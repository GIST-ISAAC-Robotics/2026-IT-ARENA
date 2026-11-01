"""고속 코너의 회랑 재확인 실패를 저장 입력으로 진단한다. 출력은 바꾸지 않는다."""
from hashlib import sha256
import json
from pathlib import Path
import sys
from zipfile import ZipFile, ZIP_DEFLATED
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy import corridor_observer as module
from arena_autonomy.follow_trace import replay_events, freeze
from arena_autonomy.local_follow import path_agrees, path_agrees_arc
from arena_autonomy.scene_motion import transport_points

original_evaluate=module.CorridorObserver.evaluate
original_proposal=module.current_proposal
captured=None
rows=[]


def proposal(*args,**kwargs):
    global captured
    captured=original_proposal(*args,**kwargs)
    return captured


def metrics(first,second,tolerance):
    a,b=[np.asarray(p) for p in (first,second)]
    selected=[p[(p[:,0]>=.05)&(p[:,0]<=.65)] for p in (a,b)]
    out=dict(path_lengths=[len(a),len(b)],x_window_counts=[len(p) for p in selected],
             x_window_spans=[float(np.ptp(p[:,0])) if len(p) else 0. for p in selected],
             tolerance_m=tolerance,
             agrees_x=bool(path_agrees(a,b,tolerance)),agrees_arc=bool(path_agrees_arc(a,b,tolerance)))
    if all(len(p) for p in selected):
        lo=max(float(p[:,0].min()) for p in selected);hi=min(float(p[:,0].max()) for p in selected)
        out['common_x_span_m']=hi-lo
        common=[p[(p[:,0]>=lo-1e-12)&(p[:,0]<=hi+1e-12)] for p in selected]
        if all(len(p) for p in common):
            distance=np.linalg.norm(common[0][:,None,:]-common[1][None,:,:],axis=2)
            out['common_x_p80_m']=float(max(np.percentile(distance.min(axis=0),80),np.percentile(distance.min(axis=1),80)))
    return out


def observe(self,scene,history,speed,side,previous_steering=0.,dt=.02,**kwargs):
    prior=None if self.path is None else self.path.copy()
    prior_ref,prior_source=self.path_ref,self.path_source
    result=original_evaluate(self,scene,history,speed,side,previous_steering,dt,**kwargs)
    if not 20.<=scene.t_ref_s<=25.:return result
    geometry=captured.get('geometry') or {}
    current=geometry.get('path')
    row=dict(reference_s=scene.t_ref_s,source_first_s=scene.source_first_s,speed_mps=speed,
             result={k:v for k,v in result.items() if k!='path'},
             geometry_reason=geometry.get('reason'),geometry_supported=geometry.get('supported'),
             prior_ref=prior_ref,prior_source=prior_source)
    if current is not None:
        row['current_path']=np.asarray(current).tolist()
        if prior is not None:
            moved=transport_points(prior,history,prior_ref,scene.t_ref_s)
            row['transported_prior']=moved.tolist()
            row['metrics']=metrics(current,moved,self.config.boundary_agreement_m)
    rows.append(row)
    return result


if __name__=='__main__':
    source=HERE.parent/'solo_headway_v1/run1/follow_decisions.jsonl'
    files=[Path(__file__).resolve(), *sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes={p.relative_to(ROOT).as_posix():sha256(p.read_bytes()).hexdigest() for p in files}
    with ZipFile(HERE/'corridor_probe_source.zip','x',ZIP_DEFLATED) as archive:
        for p in files:archive.write(p,p.relative_to(ROOT).as_posix())
    before=sha256(source.read_bytes()).hexdigest()
    module.current_proposal=proposal;module.CorridorObserver.evaluate=observe
    try:
        with source.open(encoding='utf-8') as stream:result=replay_events(json.loads(line) for line in stream)
    finally:
        module.current_proposal=original_proposal;module.CorridorObserver.evaluate=original_evaluate
    unchanged=all(sha256((ROOT/name).read_bytes()).hexdigest()==h for name,h in hashes.items())
    output=dict(scope='원래 전체 출력 재현 후 회랑 내부 비교만 수집. 호길이 대안은 출력에 연결하지 않음.',
        replay={k:v for k,v in result.items() if k!='outcomes'},source_sha256=hashes,
        source_unchanged=unchanged,input_sha256=before,input_unchanged=before==sha256(source.read_bytes()).hexdigest(),rows=rows)
    with (HERE/'corridor_probe.json').open('x',encoding='utf-8') as stream:json.dump(freeze(output),stream,ensure_ascii=False,indent=2,allow_nan=False)
    failed=[r for r in rows if r.get('metrics') and not r['metrics']['agrees_x']]
    print(json.dumps(dict(calls=result['calls'],matched=result['matched'],source_unchanged=unchanged,
        rows=len(rows),x_disagreement=len(failed),arc_agrees=sum(r['metrics']['agrees_arc'] for r in failed),
        fresh_x_disagreement=sum(r['result']['fresh'] for r in failed)),ensure_ascii=False))
    if not result['passed'] or not unchanged or not output['input_unchanged']:raise SystemExit(1)

"""중앙 통로 후보의 요소별 폐루프 비교. 이상 bicycle 합성이며 Gazebo/실차가 아니다.

표적 통과/복귀 판정은 평가 전용 정답 자세로만 계산하고 제어기에 전달하지 않는다.
"""
from collections import Counter
import hashlib
import json
import multiprocessing as mp
from pathlib import Path
import sys
import time
from unittest.mock import patch
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]


def summarize(case,r,log):
    rs=r['records']
    series=[]
    for i,x in enumerate(rs):
        e=x['evaluation'];c=e['center_pose'];use=log[i] if i<len(log) else {}
        series.append(dict(t=x['t'],x=round(c[0],4),y=round(c[1],4),yaw=round(c[2],4),v=round(x['actual_speed_mps'],4),
            cmd=round(x['command'][0],4),steer=round(x['command'][1],4),safe=round(x['safe'][0],4),reason=x['reason'],
            phase=x['entry_phase'],sel=None if not x['selected'] else [x['selected']['offset_m'],x['selected']['transition_m']],
            gap=round(e['target_body_gap_m'],4),gmin=round(e['minimum_body_gap_m'],4),grass=e['wheels_touching_grass'],
            ms=round(x['processing_ms'],2),use=use))
    target_x=case.get('target_x',1.2)
    passed=next((s['t'] for s in series if s['x']-.1>target_x+.1),None) if not case.get('restart') else None
    rejoin=next((s['t'] for s in series if s['phase']=='rejoin'),None)
    centered=next((s['t'] for s in series if s['phase']=='centered'),None)
    best=0.;start=None
    for s in series:
        if s['v']<.02 and s['t']>.9:
            start=s['t'] if start is None else start;best=max(best,s['t']-start)
        else:start=None
    ms=sorted(s['ms'] for s in series)
    return series,dict(case=case,final_x=series[-1]['x'],final_y=series[-1]['y'],contact=r['contact_samples'],
        gap_min=r['minimum_body_gap_m'],target_gap_min=min(s['gap'] for s in series),grass_max=r['maximum_wheels_touching_grass'],
        stop_zero=r['stop_command_all_zero'],final_speed=r['final_speed_mps'],passed_t=passed,rejoin_t=rejoin,
        centered_t=centered,last_phase=series[-1]['phase'],longest_stall_s=best,
        first_bypass=next((dict(t=x['t'],offset=x['selected']['offset_m'],transition=x['selected']['transition_m']) for x in rs if x['selected'] and x['selected']['offset_m']),None),
        reasons=r['reasons'],safety=dict(Counter(x['safety_reason'] for x in rs)),
        ms_mean=sum(ms)/len(ms),ms_p95=ms[int(.95*len(ms))],ms_max=ms[-1])


def run(case):
    began=time.perf_counter()
    import probe_bypass_entry_sequence as probe
    from arena_autonomy.bypass_entry_trial import EntryGuide
    from candidate import CorridorGuide,CorridorPolicy
    log=[]
    if case.get('policy') is None:
        factory=EntryGuide
    else:
        policy=CorridorPolicy(**case['policy'])
        class Logged(CorridorGuide):
            def evaluate(self,*args,**kwargs):
                out=super().evaluate(*args,**kwargs)
                u=out.get('corridor',{}).get(self.side) if self.side else None
                log.append(dict(side=self.side,inner=None if not u or not u['surface'] else round(u['surface']['normal_m'],4),
                    method=None if not u or not u['surface'] else u['surface']['method'],
                    pad=None if not u or u['maximum_inner_padding_m'] is None else round(u['maximum_inner_padding_m'],4),
                    wall=0 if not u else u['wall_rays_aligned'],aligned=0 if not u else u['inner_rays_aligned'],onset=0 if not u else u['onset_rays'],
                    stable=bool(u and u['state'].get('stable')),approach=None if not u else u['state'].get('approach_m'),
                    supported=0 if not u else u['supported_rays'],supported_along=None if not u else u['supported_along_m'],
                    plan=out.get('corridor_plan') if self.path is None else None))
                return out
        factory=lambda config,entry,**kw:Logged(config,entry,corridor_policy=policy,**kw)
    boxes=probe.evaluation_boxes
    if case.get('scene') is not None:
        from scenes import make_boxes
        boxes=make_boxes(case['scene'])
    extra=[]
    if case.get('bias') is not None:
        from scenes import biased
        extra.append(patch.object(probe,'ray_box_distance',biased(probe.ray_box_distance,case['bias'])))
    if case.get('noise_half_width') is not None:
        width=case['noise_half_width']
        extra.append(patch.object(probe.ProbeSensors,'range_value',lambda self,distance:distance if distance==float('inf')
            else distance+float(self.range_rng.uniform(-width,width))))
    for item in extra:
        item.start()
    with patch.object(probe,'EntryGuide',factory),patch.object(probe,'evaluation_boxes',boxes):
        r=probe.rollout(case.get('target_y',0.),case.get('restart',False),case.get('duration',20.),True,True,True,True,
            case.get('sensor','combined'),case.get('seed',17),True,True,False)
    for item in extra:
        item.stop()
    series,summary=summarize(case,r,log)
    summary['wall_seconds']=time.perf_counter()-began
    out=BASE/case['batch']/case['name'];out.mkdir(parents=True,exist_ok=False)
    (out/'series.json').write_text(json.dumps(series)+'\n',encoding='utf-8')
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    return summary


def main(batch,cases):
    paths=list((ROOT/'src').rglob('*.py'))+[Path(__file__),BASE/'candidate.py',BASE/'scenes.py']+list((ROOT/'scripts').glob('*bypass*.py'))
    paths+=[ROOT/'scripts/g2_tracking_probe.py',ROOT/'scripts/g0_geometry.py',ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_shadow.py']
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    (BASE/batch).mkdir(exist_ok=False)
    with zipfile.ZipFile(BASE/batch/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in paths:archive.write(p,p.relative_to(ROOT))
    results=[]
    with mp.get_context('spawn').Pool(12,maxtasksperchild=1) as pool:
        for s in pool.imap_unordered(run,[dict(c,batch=batch) for c in cases]):
            results.append(s)
            print(s['case']['name'],s['final_x'],s['last_phase'],s['passed_t'],s['centered_t'],flush=True)
    unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    (BASE/batch/'report.json').write_text(json.dumps(dict(cases=sorted(results,key=lambda s:s['case']['name']),source_sha256=hashes,source_unchanged=unchanged),indent=2)+'\n',encoding='utf-8')
    if not unchanged:raise ValueError('source changed')

"""진단 전용: 표적 점의 여유만 바꾼 중앙 폐루프. 운영 후보가 아니다.

벽으로 분류된 반환의 55 mm, 경로 생성, 추종, 관측, 독립 보호, STOP은 그대로다.
'constant' 모드는 나이와 무관한 고정 여유로 폐루프가 요구하는 통로 폭을 재는 도구이며
그 값을 안전 여유로 제안하지 않는다.
"""
from collections import Counter
import hashlib
import json
import multiprocessing as mp
from pathlib import Path
import sys
import time
import zipfile

import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
FIXED=.055
CASES=[
    dict(name='bound_0.50',mode='bound',v=.5),
    dict(name='bound_0.25',mode='bound',v=.25),
    dict(name='bound_0.10',mode='bound',v=.10),
    dict(name='bound_0.00',mode='bound',v=0.),
    dict(name='const_055',mode='constant',p=.055),
    dict(name='const_070',mode='constant',p=.070),
    dict(name='const_085',mode='constant',p=.085),
    dict(name='const_100',mode='constant',p=.100),
    dict(name='const_115',mode='constant',p=.115),
    dict(name='const_130',mode='constant',p=.130),
    dict(name='bound_0.00_early',mode='bound',v=0.,early=True),
    dict(name='const_100_early',mode='constant',p=.100,early=True),
    dict(name='bound_0.00_seed18',mode='bound',v=0.,seed=18),
    dict(name='bound_0.00_seed19',mode='bound',v=0.,seed=19),
]


def install(case):
    import arena_autonomy.bypass_entry_trial as bt
    original=bt.current_outer_wall_padding
    def patched(scene,config,side,wall,growth_s=0.):
        padding,evidence=original(scene,config,side,wall,growth_s)
        padding=np.asarray(padding,dtype=float).copy()
        rays=[r for r in scene.rays if r.status=='return']
        ages=np.asarray([scene.t_ref_s-r.acquired_at_s for r in rays],dtype=float)
        target=padding>FIXED+1e-12  # 벽 분류 점은 원래 FIXED다.
        if case['mode']=='bound':padding[target]=FIXED+case['v']*ages[target]
        else:padding[target]=case['p']
        return padding,evidence
    bt.current_outer_wall_padding=patched


def run(case):
    began=time.perf_counter()
    install(case)
    import probe_bypass_entry_sequence as probe
    r=probe.rollout(0.,False,20.,True,True,True,True,'combined',case.get('seed',17),True,True,bool(case.get('early',False)))
    rs=r['records']
    series=[dict(t=x['t'],x=round(x['evaluation']['center_pose'][0],4),y=round(x['evaluation']['center_pose'][1],4),
        yaw=round(x['evaluation']['center_pose'][2],4),v=round(x['actual_speed_mps'],4),cmd=round(x['command'][0],4),
        steer=round(x['command'][1],4),safe=round(x['safe'][0],4),reason=x['reason'],phase=x['entry_phase'],
        sel=None if not x['selected'] else [x['selected']['offset_m'],x['selected']['transition_m']],
        gap=round(x['evaluation']['target_body_gap_m'],4),gmin=round(x['evaluation']['minimum_body_gap_m'],4),
        grass=x['evaluation']['wheels_touching_grass']) for x in rs]
    passed=next((s['t'] for s in series if s['x']-.1>1.3),None)  # 자차 뒤면이 표적 앞면 통과(평가 전용)
    rejoin=next((s['t'] for s in series if s['phase']=='rejoin'),None)
    centered=next((s['t'] for s in series if s['phase']=='centered'),None)
    summary=dict(case=case,final_x=series[-1]['x'],final_y=series[-1]['y'],contact=r['contact_samples'],
        gap_min=r['minimum_body_gap_m'],grass_max=r['maximum_wheels_touching_grass'],stop_zero=r['stop_command_all_zero'],
        final_speed=r['final_speed_mps'],passed_t=passed,rejoin_t=rejoin,centered_t=centered,last_phase=series[-1]['phase'],
        first_bypass=next((dict(t=x['t'],sel=x['selected']) for x in rs if x['selected'] and x['selected']['offset_m']),None),
        reasons=r['reasons'],safety=dict(Counter(x['safety_reason'] for x in rs)),
        target_gap_min=min(s['gap'] for s in series),wall_seconds=time.perf_counter()-began)
    out=BASE/'closed1'/case['name'];out.mkdir(parents=True,exist_ok=False)
    (out/'series.json').write_text(json.dumps(series)+'\n',encoding='utf-8')
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    return summary


if __name__=='__main__':
    paths=list((ROOT/'src').rglob('*.py'))+[Path(__file__)]+list((ROOT/'scripts').glob('*bypass*.py'))
    paths+=[ROOT/'scripts/g2_tracking_probe.py',ROOT/'scripts/g0_geometry.py',ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_shadow.py']
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    (BASE/'closed1').mkdir(exist_ok=False)
    with zipfile.ZipFile(BASE/'closed1/sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in paths:archive.write(p,p.relative_to(ROOT))
    with mp.get_context('spawn').Pool(8) as pool:
        results=[]
        for s in pool.imap_unordered(run,CASES):
            results.append(s);print(json.dumps({k:s[k] for k in ('final_x','final_y','contact','gap_min','grass_max','passed_t','rejoin_t','centered_t','last_phase','wall_seconds')}|{'name':s['case']['name']}),flush=True)
    unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    (BASE/'closed1/report.json').write_text(json.dumps(dict(cases=sorted(results,key=lambda s:s['case']['name']),source_sha256=hashes,source_unchanged=unchanged),indent=2)+'\n',encoding='utf-8')
    if not unchanged:raise ValueError('source changed')

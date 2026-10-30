"""통과 중 측방 침범의 시점 분해와 기존 팽창 판정 후향 대조.

같은 자세, 같은 스캔에서 기존 여유(0.5 m/s*age)의 판정을 그림자로 함께 계산한다.
그림자는 출력에 쓰지 않는다. 진입 뒤 기존 판정이 줄곧 정지였다면 그 노출은 후보가 새로 만든 것이다.
"""
import hashlib
import json
import multiprocessing as mp
from pathlib import Path
import sys
from unittest.mock import patch
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
B=dict(surface=True,plan=True,onset=True)
CASES=[]
for start in (3.5,4.3,4.8):
    for speed in (.05,.15,.25,.5):
        CASES.append(dict(name=f'slide_t{start}_u{speed}',scene=dict(kind='cutin',start=start,speed=speed,distance=.2,direction=-1.)))
for start in (3.5,4.3,4.8):
    CASES.append(dict(name=f'pullout_t{start}',scene=dict(kind='pullout',start=start)))
CASES.append(dict(name='control_no_motion',scene=dict(kind='fixed')))


def analyze(case,rows):
    start=case['scene'].get('start',.5)
    after=[r for r in rows if r['t']>=start]
    first=lambda test:next((r for r in after if test(r)),None)
    def mark(r):
        return None if r is None else dict(t=r['t'],x=r['x'],y=r['y'],speed=r['v'],command=r['cmd'],target_gap_m=r['gap'],minimum_gap_m=r['gmin'])
    onset=rows[min(range(len(rows)),key=lambda i:abs(rows[i]['t']-start))]
    detect=first(lambda r:not r['stable'] or r['approach'] is not None and r['approach']>.03)
    zero=first(lambda r:r['cmd']==0.)
    stop=first(lambda r:r['cmd']==0. and r['v']<.005)
    contact=rows[-1] if rows[-1]['gap']<=1e-10 else None
    entered=[r for r in rows if r['sel'] is not None and r['t']<=start and r['x']>.7]
    shadow=[r for r in rows if r['shadow'] is not None]
    held=[r for r in shadow if not r['shadow']['eligible']]
    return dict(case=case,onset=mark(onset),detect=mark(detect),zero_command=mark(zero),ego_stopped=mark(stop),contact=mark(contact),
        ticks_contact_after_stop=None if not stop or not contact else round((contact['t']-stop['t'])/.02),
        final_x=rows[-1]['x'],final_y=rows[-1]['y'],minimum_target_gap_m=min(r['gap'] for r in rows),
        passed=any(r['x']-.1>1.3 for r in rows),
        shadow_compared=len(shadow),shadow_existing_would_hold=len(held),
        shadow_first_hold=mark(held[0]) if held else None,
        shadow_hold_before_onset=sum(r['t']<start for r in held),candidate_moving_ticks_before_onset=len(entered))


def run(case):
    import probe_bypass_entry_sequence as probe
    import arena_autonomy.bypass_entry_trial as bt
    from candidate import CorridorGuide,CorridorPolicy
    from scenes import make_boxes
    policy=CorridorPolicy(**B);log=[]
    class Shadow(CorridorGuide):
        def evaluate(self,scene,history,speed,previous,dt):
            out=super().evaluate(scene,history,speed,previous,dt)
            u=out.get('corridor',{}).get(self.side) if self.side else None
            shadow=None
            if self.path is not None and out.get('reference') and self.side in out['reference'].get('walls',{}):
                # 경로와 자세가 같을 때 원래 여유로 다시 판정한다. 상태를 바꾸지 않는 순수 비교다.
                compared=bt._compare_entries(scene,scene.t_ref_s,speed,previous,dt,config=self.config,entry=self.entry,
                    guide=dict(reference=out['reference'],path=self.path,offset_m=self.offset,transition_m=self.transition,
                        outer_side=self.side,stationary_outer_boundary=self.wall_confirmations>=2))
                row=compared['candidates'][0] if compared['candidates'] else {}
                shadow=dict(eligible=bool(row.get('eligible')),reason=row.get('reason'),clearance_m=row.get('command_clearance_m'),
                    speed_mps=row.get('target_speed_mps'))
            log.append(dict(stable=bool(u and u['state'].get('stable')),approach=None if not u else u['state'].get('approach_m'),
                inner=None if not u or not u['surface'] else u['surface']['normal_m'],shadow=shadow))
            return out
    factory=lambda config,entry,**kw:Shadow(config,entry,corridor_policy=policy,**kw)
    with patch.object(probe,'EntryGuide',factory),patch.object(probe,'evaluation_boxes',make_boxes(case['scene'])):
        r=probe.rollout(0.,False,14.,True,True,True,True,'combined',17,True,True,False)
    rows=[]
    for x,u in zip(r['records'],log):
        c=x['evaluation']['center_pose']
        rows.append(dict(t=x['t'],x=round(c[0],4),y=round(c[1],4),v=round(x['actual_speed_mps'],4),cmd=round(x['command'][0],4),
            safe=round(x['safe'][0],4),sel=None if not x['selected'] else x['selected']['offset_m'],
            gap=round(x['evaluation']['target_body_gap_m'],4),gmin=round(x['evaluation']['minimum_body_gap_m'],4),
            phase=x['entry_phase'],reason=x['reason'],**u))
    out=BASE/'timeline1'/case['name'];out.mkdir(parents=True,exist_ok=False)
    (out/'series.json').write_text(json.dumps(rows)+'\n',encoding='utf-8')
    result=analyze(case,rows)
    (out/'timeline.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    return result


if __name__=='__main__':
    paths=[Path(__file__),BASE/'candidate.py',BASE/'scenes.py']
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    (BASE/'timeline1').mkdir(exist_ok=False)
    with zipfile.ZipFile(BASE/'timeline1/sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in paths:
            archive.write(p,p.relative_to(ROOT))
    results=[]
    with mp.get_context('spawn').Pool(12,maxtasksperchild=1) as pool:
        for s in pool.imap_unordered(run,CASES):
            results.append(s);print(s['case']['name'],s['final_x'],bool(s['contact']),s['passed'],flush=True)
    unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    (BASE/'timeline1/report.json').write_text(json.dumps(dict(cases=sorted(results,key=lambda s:s['case']['name']),source_sha256=hashes,source_unchanged=unchanged),indent=2)+'\n',encoding='utf-8')

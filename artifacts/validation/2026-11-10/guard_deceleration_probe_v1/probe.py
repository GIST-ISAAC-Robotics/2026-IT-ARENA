"""실제 판단은 그대로 재현하면서 GUARD 감속 목표만 비교한다. 실행 연결 없음."""
from collections import Counter
import hashlib,json,sys,zipfile
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
from arena_autonomy.pass_release_trial import PassReleaseSelector,motion_region,region_blocked
from arena_autonomy.measured_parallel_speed_trial import proposal
from arena_autonomy import m3_execution_trial as mx


def main():
    out=A/'result';out.mkdir(exist_ok=False)
    trace=A.parent/'cut_in_recessed_facility_v1/close_cut_in/run1/follow_decisions.jsonl'
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in sources:z.write(ROOT/p,p)
        z.write(Path(__file__),Path(__file__).relative_to(ROOT))
    digest=sha(trace);original=PassReleaseSelector.select;rows=[];seen=set();counts=Counter()
    def wrapped(self,scene,baseline,parallel,speed,previous,dt,**kw):
        actual=original(self,scene,baseline,parallel,speed,previous,dt,**kw)
        if not (self.mode=='GUARD' and actual[0]>0 and actual[0]<=.35+1e-9
                and actual[2].get('reason')=='parallel_hold_follow' and speed>.45
                and baseline[0]>actual[0]+1e-6 and self.follow_config.piecewise_wall_trial
                and scene.source_first_s not in seen):return actual
        seen.add(scene.source_first_s);attempts=[]
        for cap in (.5,.7,1.):
            candidate=proposal(scene,baseline,speed,previous,dt,self.follow_config,min(cap,baseline[0]))
            item=dict(cap_mps=cap,proposal=candidate,accepted=False)
            if candidate['eligible'] and candidate['target_speed_mps']>actual[0]+1e-6:
                target=candidate['target_speed_mps'];steer=candidate['steering_rad']
                region=motion_region(scene,speed,target,steer,previous,dt,self.follow_config,self.lat)
                blocked=region_blocked(region,self.config)
                geometry=mx.arc_checks(scene,[],baseline[2]['path_points'],(steer,previous),
                    region['distance_m'],self.lat,mx.ExecutionConfig())
                item.update(region=region,blocked=blocked,geometry=geometry['geometry'],
                    accepted=blocked is None and geometry['geometry_ok'])
            attempts.append(item)
        rows.append(freeze(dict(t=scene.t_ref_s,source=scene.source_first_s,measured=speed,
            actual=actual[:2],baseline=baseline[:2],attempts=attempts)))
        return actual
    PassReleaseSelector.select=wrapped
    try:
        for e in read_events(trace):
            d=e['data']
            if e['kind']=='header':ctl,mc=build_controller(d)
            elif e['kind']=='reset':ctl.reset()
            elif e['kind']=='call':
                result=freeze(replay_call(ctl,d['input'],mc));counts['calls']+=1
                counts['mismatch']+=not equal_output(result[:2],d['output'][:2])
    finally:PassReleaseSelector.select=original
    summary=dict(counts=dict(counts),rows=rows,input_sha256=digest,input_unchanged=sha(trace)==digest,
        sources=sources,source_unchanged=all(sha(ROOT/p)==h for p,h in sources.items()),
        scope='read-only; no command selection or state propagation from proposal')
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    brief=[dict(t=r['t'],speed=r['measured'],accepted=[x['proposal']['target_speed_mps'] for x in r['attempts'] if x['accepted']],
        reasons=[x.get('blocked') or x['proposal']['reason'] for x in r['attempts']]) for r in rows]
    print(json.dumps(dict(counts=counts,rows=brief,source_unchanged=summary['source_unchanged'])))
    assert counts['mismatch']==0 and summary['input_unchanged'] and summary['source_unchanged']

if __name__=='__main__':main()

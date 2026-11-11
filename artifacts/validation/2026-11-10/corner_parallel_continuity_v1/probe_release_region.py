"""실패 주행의 RELEASED 관측 영역을 그대로 재현하고 낮은 목표에서 별도 검사한다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
import arena_autonomy.pass_release_trial as ps


def main():
    path=A/'run1/follow_decisions.jsonl'
    out=A/'release_region_probe';out.mkdir(exist_ok=False)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in [path,*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]}
    original=ps.PassReleaseSelector.select
    captures=[];seen=set()
    def select(self,scene,baseline,parallel,speed,previous,dt,**kw):
        result=original(self,scene,baseline,parallel,speed,previous,dt,**kw)
        if (result[2].get('selector_reason')=='large_motion_region_unobserved'
                and self.mode=='RELEASED' and scene.source_first_s not in seen):
            seen.add(scene.source_first_s)
            row=dict(t=scene.t_ref_s,source_s=scene.source_first_s,measured_speed_mps=speed,
                baseline_speed_mps=baseline[0],steering_rad=baseline[1],previous_steering_rad=previous,
                path_current=baseline[2].get('boundary_from_memory') is False
                    and baseline[2].get('path_source_s')==scene.source_first_s,
                original_region=result[2].get('selector_region'),sides=self.sides,
                candidates=[])
            for target in (.35,.20):
                if baseline[0]<=target:continue
                context={}
                region=ps.motion_region(scene,speed,target,baseline[1],previous,dt,self.follow_config,self.lat,context=context)
                unknown=(context['queries'][~context['free'] & ~context['masked']]
                         if context else np.empty((0,2)))
                row['candidates'].append(dict(target_mps=target,prediction_speed_mps=max(speed,target),
                    region=region,blocked=ps.region_blocked(region,self.config),
                    unknown_bounds=None if not len(unknown) else [unknown.min(axis=0),unknown.max(axis=0)],
                    unknown_points=unknown))
            captures.append(freeze(row))
        return result
    ps.PassReleaseSelector.select=select
    counts=Counter()
    try:
        for event in read_events(path):
            data=event['data']
            if event['kind']=='header':controller,motion=build_controller(data)
            elif event['kind']=='reset':controller.reset()
            elif event['kind']=='call':
                result=freeze(replay_call(controller,data['input'],motion))
                counts['calls']+=1
                counts['command_mismatch']+=not equal_output(result[:2],data['output'][:2])
    finally:ps.PassReleaseSelector.select=original
    with (out/'captures.jsonl').open('x') as f:
        for row in captures:f.write(json.dumps(row)+'\n')
    summary=[]
    for row in captures:
        clean={**row,'candidates':[{k:v for k,v in c.items() if k!='unknown_points'} for c in row['candidates']]}
        summary.append(clean)
    result=dict(scope='readonly instrumented replay; lower target keeps measured speed and all region gates; not a controller change',
        counts=dict(counts),captured_scans=len(captures),rows=summary,source_sha256=sources,
        source_unchanged=all(sha(ROOT/p)==h for p,h in sources.items()))
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    viable=Counter(str(c['target_mps']) for r in summary for c in r['candidates'] if c['blocked'] is None)
    print(json.dumps(dict(counts=dict(counts),captured=len(captures),viable_by_target=dict(viable),source_unchanged=result['source_unchanged'])))
    assert counts['command_mismatch']==0


if __name__=='__main__':main()

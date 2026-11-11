"""코너 정지의 이전 실제 광선 지지를 조사한다. 명령/판정은 전혀 바꾸지 않는다."""
from collections import Counter
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
from arena_autonomy.local_follow import LocalFollow
from arena_autonomy.observation_support import depth_support,source_valid
from arena_autonomy.observation_lateral import reachable_from_unknown
import arena_autonomy.pass_release_trial as ps


def main():
    out=A/'recent_rear_support_probe';out.mkdir(exist_ok=False)
    path=A/'run1/follow_decisions.jsonl'
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in
        [path,*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]}
    original_forward=LocalFollow._forward_command;original_select=ps.PassReleaseSelector.select
    caches={};available={};seen=set();rows=[];counts=Counter()
    def forward(self,scene,history,speed,side,previous_steering=0.,control_dt=.02,*,raw_scan=None):
        if self.pass_selector is not None:
            key=id(self.pass_selector)
            if key not in caches:caches[key]=ps.mx.SceneHistory()
            available[key]=caches[key].update(scene,history)
        return original_forward(self,scene,history,speed,side,previous_steering,control_dt,raw_scan=raw_scan)

    def select(self,scene,baseline,parallel,speed,previous,dt,**kw):
        result=original_select(self,scene,baseline,parallel,speed,previous,dt,**kw)
        if (len(rows)>=6 or result[2].get('selector_reason')!='large_motion_region_unobserved'
                or self.mode!='RELEASED' or scene.source_first_s in seen):return result
        seen.add(scene.source_first_s)
        context={};target=min(.35,baseline[0])
        region=ps.motion_region(scene,speed,target,baseline[1],previous,dt,
            self.follow_config,self.lat,context=context)
        if not context or not region['unobserved']:return result
        q=context['queries'][~context['free'] & ~context['masked']]
        rear=(np.all(q[:,0]>=-self.follow_config.length_m/2) and np.all(q[:,0]<=0)
            and np.all(np.abs(q[:,1])>self.follow_config.width_m/2)
            and np.all(np.abs(q[:,1])-self.follow_config.width_m/2<=.01))
        hits=np.array([r.hit_xy_m for r in scene.rays if r.status=='return']).reshape(-1,2)
        hitpad=context['padding'][np.array([r.status=='return' for r in scene.rays])]
        occupied=np.any(np.linalg.norm(q[:,None]-hits[None,:],axis=2)<=hitpad,axis=1) if len(hits) else np.zeros(len(q),bool)
        supported=np.zeros(len(q),bool);positive_union=supported.copy();frames=[]
        for old,(dx,dy,yaw) in available.get(id(self),[]):
            if (not source_valid(old) or old.source_epoch!=scene.source_epoch
                    or old.source_first_s>=scene.source_first_s):continue
            c,s=math.cos(yaw),math.sin(yaw)
            local=(q-[dx,dy])@np.array([[c,-s],[s,c]])
            current_age=replace(old,t_ref_s=scene.t_ref_s)
            pad=ps.mx.ray_padding(current_age,self.lat,self.follow_config.object_motion_bound_mps)
            positive=depth_support(old,local,pad)
            positive_union|=positive
            reachable=np.zeros(len(q),bool)
            if np.any(positive):
                reachable[positive]=reachable_from_unknown(current_age,local[positive],self.follow_config)
            accepted=positive & ~reachable & ~occupied
            supported|=accepted
            frames.append(dict(source_s=old.source_first_s,age_s=scene.t_ref_s-old.source_first_s,
                positive=int(positive.sum()),reachable_from_unknown=int((positive & reachable).sum()),
                current_occupied=int((positive & occupied).sum()),supported=int(accepted.sum())))
        rows.append(freeze(dict(t=scene.t_ref_s,speed=speed,target=target,region=region,
            rear_within_existing_one_cm=bool(rear),current_occupied=int(occupied.sum()),frames=frames,
            positive_union=int(positive_union.sum()),supported_union=int(supported.sum()),
            all_supported=bool(np.all(supported)),motion_bound_mps=self.follow_config.object_motion_bound_mps)))
        return result

    LocalFollow._forward_command=forward;ps.PassReleaseSelector.select=select
    try:
        for event in read_events(path):
            data=event['data']
            if event['kind']=='header':controller,motion=build_controller(data)
            elif event['kind']=='reset':controller.reset();caches.clear();available.clear()
            elif event['kind']=='call':
                result=freeze(replay_call(controller,data['input'],motion));counts['calls']+=1
                counts['command_mismatch']+=not equal_output(result[:2],data['output'][:2])
    finally:LocalFollow._forward_command=original_forward;ps.PassReleaseSelector.select=original_select
    result=dict(scope='read-only previous actual ray support; unchanged outputs; no proposed permission',
        counts=dict(counts),rows=rows,sources=sources,
        source_unchanged=all(sha(ROOT/p)==h for p,h in sources.items()))
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(counts=counts,rows=rows,source_unchanged=result['source_unchanged'])))
    assert counts['command_mismatch']==0 and result['source_unchanged']


if __name__=='__main__':main()

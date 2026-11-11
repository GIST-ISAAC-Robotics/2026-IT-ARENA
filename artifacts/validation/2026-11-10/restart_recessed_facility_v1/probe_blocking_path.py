"""저장 재출발 입력에서 점유 제동점과 현재벽 중앙 경로를 읽기 전용 비교한다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

A=Path(__file__).resolve().parent; ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller,read_events,Poses
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
import arena_autonomy.bypass_entry_trial as bt


def main():
    out=A/'blocking_path_probe';out.mkdir(exist_ok=False)
    source=A/'moving_stop_restart/run1/follow_decisions.jsonl'
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in
        [source,*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]}
    poses=Poses(A/'moving_stop_restart/run1/ego_poses.jsonl')
    original=bt._compare_entries; rows=[]; counts=Counter(); thresholds=[12.8,13.4,14.4,16.,20.]
    def compare(scene,now,speed,previous,dt,**kw):
        guide=kw.get('guide'); snapshot={}; provider=kw.get('padding_provider')
        kwargs=dict(kw)
        if provider is not None:
            def padding(*args):
                result=provider(*args); snapshot['padding']=result; return result
            kwargs['padding_provider']=padding
        result=original(scene,now,speed,previous,dt,**kwargs)
        if not thresholds or now<thresholds[0] or guide is None or 'padding' not in snapshot:
            return result
        thresholds.pop(0)
        points=np.asarray(scene.occupied_endpoints_xy_m); pad=snapshot['padding'][0]
        config=kw['config']; entry=kw['entry']; wall=guide['reference']['walls'][guide['outer_side']]
        fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
        primary=result.get('candidates',[{}])[0]; steer=primary.get('steering_rad',previous)
        individual=[min(bt.swept_limit(np.asarray([p]),angle,config.wheelbase_m,horizon=2.5,
            margin=np.asarray([margin]),length=config.length_m,width=config.width_m)
            for angle in (steer,previous)) for p,margin in zip(points,pad)]
        ids=np.argsort(individual)[:12]; x,y,yaw=poses.at(now)
        rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        world=points@rotation.T+np.array([x,y])
        returns=[r for r in scene.rays if r.status=='return']
        row=dict(t=now,measured_speed=speed,previous=previous,pose=[x,y,yaw],guide=guide,
            primary=primary,blocking_points=[dict(ray=returns[i].source_index,local=points[i],
                diagnostic_world=world[i],padding=pad[i],clearance=individual[i]) for i in ids],alternatives=[])
        for offset in (0.,guide['offset_m']/2):
            for transition in entry.transitions_m:
                try:path=bt.entry_path(fit,offset,config,entry,transition)
                except ValueError:continue
                alternative=original(scene,now,speed,previous,dt,config=config,entry=entry,
                    guide=dict(guide,path=path,offset_m=offset,transition_m=transition),
                    padding_provider=lambda *args:snapshot['padding'])
                row['alternatives'].append(dict(offset=offset,transition=transition,result=alternative))
        rows.append(freeze(row)); return result
    bt._compare_entries=compare
    try:
        for event in read_events(source):
            data=event['data']
            if event['kind']=='header': controller,motion=build_controller(data)
            elif event['kind']=='reset':controller.reset()
            elif event['kind']=='call':
                result=freeze(replay_call(controller,data['input'],motion));counts['calls']+=1
                counts['command_mismatch']+=not equal_output(result[:2],data['output'][:2])
    finally:bt._compare_entries=original
    result=dict(scope='read-only path diagnosis; truth poses used only for labeling points after controller checks',
        counts=dict(counts),rows=rows,sources=sources,source_unchanged=all(sha(ROOT/p)==h for p,h in sources.items()))
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(counts=counts,source_unchanged=result['source_unchanged'],rows=[dict(t=r['t'],
        primary_clearance=r['primary'].get('command_clearance_m'),blockers=r['blocking_points'][:2],
        alternatives=[dict(offset=a['offset'],transition=a['transition'],selected=a['result']['selected'],
            reasons=[c['reason'] for c in a['result']['candidates']]) for a in r['alternatives']]) for r in rows])))
    assert counts['command_mismatch']==0 and result['source_unchanged']


if __name__=='__main__':main()

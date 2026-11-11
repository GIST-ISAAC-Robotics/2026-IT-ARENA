"""r104 저장 입력의 경계 전환 조사. 런타임/판정 변경 없이 전체 재생한다."""
import json
from pathlib import Path
import sys
import numpy as np

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy import local_follow as lf
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import freeze,replay_call,equal_output,comparable_decision
from arena_autonomy.scene_motion import transport_points


def main():
    source=A.parent/'async_handoff_r104_v1/two_actor_run1/run1/follow_decisions.jsonl'
    rows=[json.loads(line) for line in source.read_text('utf8').splitlines()]
    follow,motion=make_controller(rows[0]['data'])
    active={};frames=[];mismatches=[];commands=[]
    original_baseline=lf.LocalFollow._baseline_command;original_path=lf.local_path
    def baseline(self,scene,history,*args,**kwargs):
        active.clear()
        if 49.65 < scene.t_ref_s < 50.47:
            carried=None
            if self.path is not None and scene.t_ref_s-self.path_source < self.config.memory_ttl_s:
                carried=transport_points(self.path,history,self.path_ref,scene.t_ref_s)
            frame=dict(reference_s=scene.t_ref_s,path_source=self.path_source,
                       path_wall=self.path_wall,reference=None if carried is None else carried.tolist(),
                       points=scene.occupied_endpoints_xy_m,paths=[])
            frames.append(frame);active.update(frame=frame,carried=carried,config=self.config)
        try:return original_baseline(self,scene,history,*args,**kwargs)
        finally:active.clear()
    def path(*args,**kwargs):
        result=original_path(*args,**kwargs)
        if active:
            candidate,info=result;carried=active['carried'];cfg=active['config']
            agreement=bool(candidate is not None and carried is not None and lf.path_agrees(candidate,carried,cfg.boundary_agreement_m))
            arc=bool(candidate is not None and carried is not None and lf.path_agrees_arc(candidate,carried,cfg.boundary_agreement_m))
            active['frame']['paths'].append(dict(side=args[1],points=len(args[0]),info=info,
                path=None if candidate is None else candidate.tolist(),x_agrees=agreement,arc_agrees=arc,
                anchor_error=lf.anchor_error(info,cfg) if candidate is not None else None))
        return result
    lf.LocalFollow._baseline_command=baseline;lf.local_path=path
    try:
        for row in rows[1:]:
            if row['kind']=='reset':follow.reset()
            elif row['kind']=='call':
                data=row['data'];actual=freeze(replay_call(follow,data['input'],motion))
                if not equal_output(comparable_decision(actual),comparable_decision(data['output'])):
                    mismatches.append(row['seq'])
                if not equal_output(actual[:2],data['output'][:2]):commands.append(row['seq'])
    finally:lf.LocalFollow._baseline_command=original_baseline;lf.local_path=original_path
    result=dict(calls=sum(r['kind']=='call' for r in rows),mismatches=mismatches,command_mismatches=commands,frames=freeze(frames))
    with (A/'probe.json').open('x',encoding='utf8') as f:json.dump(result,f,ensure_ascii=False)
    compact=[{**{k:v for k,v in row.items() if k not in ('reference','points','paths')},
        'paths':[{k:v for k,v in p.items() if k!='path'} for p in row['paths']]} for row in result['frames']]
    with (A/'summary.json').open('x',encoding='utf8') as f:json.dump(dict(calls=result['calls'],mismatches=mismatches,command_mismatches=commands,frames=compact),f,indent=2)
    print(json.dumps(dict(calls=result['calls'],mismatches=mismatches,command_mismatches=commands,frames=len(frames))))

if __name__=='__main__':main()

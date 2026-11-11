"""r105 관측/측방 가림 진단. 제어 변경 없이 재생의 같은 후보 질의를 회수한다."""
import hashlib,json,sys
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy import pass_release_trial as pr
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import freeze,replay_call,equal_output,comparable_decision


def main():
    source=A.parent/'async_corner_r105_v1/two_actor_run1/run1/follow_decisions.jsonl'
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    rows=[json.loads(line) for line in source.read_text('utf8').splitlines()]
    follow,motion=make_controller(rows[0]['data'])
    original=pr.observed_guard_approach;original_select=pr.PassReleaseSelector._select
    active={};frames=[];mismatch=[]
    chosen={636,640,644,645,650,700}
    def select(self,scene,baseline,parallel,*args,**kwargs):
        if active.get('seq') in chosen:
            active.update(scene=scene,path=baseline[2].get('path_points'),side=self.lat)
        try:return original_select(self,scene,baseline,parallel,*args,**kwargs)
        finally:
            for key in ('scene','path','side'):active.pop(key,None)
    def approach(frame,context,sides,rejected):
        result=original(frame,context,sides,rejected)
        if active.get('seq') in chosen and 'scene' in active:
            scene=active['scene']
            frames.append(freeze(dict(seq=active['seq'],reference_s=scene.t_ref_s,
                points=scene.occupied_endpoints_xy_m,path=active['path'],
                queries=context['queries'],free=context['free'],masked=context.get('masked'),
                guarded_sides=sides,rejected_sides=rejected,result=result)))
        return result
    pr.PassReleaseSelector._select=select;pr.observed_guard_approach=approach
    try:
        for row in rows[1:]:
            if row['kind']=='reset':follow.reset()
            elif row['kind']=='call':
                active['seq']=row['seq']
                actual=freeze(replay_call(follow,row['data']['input'],motion))
                if not equal_output(comparable_decision(actual),comparable_decision(row['data']['output'])):
                    mismatch.append(row['seq'])
    finally:
        pr.PassReleaseSelector._select=original_select;pr.observed_guard_approach=original
    assert hashlib.sha256(source.read_bytes()).hexdigest()==digest
    with (A/'guard_probe.json').open('x',encoding='utf8') as output:
        json.dump(dict(source=source.relative_to(ROOT).as_posix(),sha256=digest,mismatches=mismatch,frames=frames),output)
    print(json.dumps(dict(mismatches=mismatch,frames=len(frames),sequences=[f['seq'] for f in frames])))


if __name__=='__main__':main()

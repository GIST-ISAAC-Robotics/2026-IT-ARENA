"""C4: 지주 점 개수와 복귀 전방 기준을 분리하는 저장 입력 비교. 실행 소스 불변."""
import hashlib,json,types
from collections import Counter
from contextlib import contextmanager
from pathlib import Path
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.bypass_handoff_corridor_trial import ForecastGuide
import arena_autonomy.bypass_entry_trial as entry
A=Path(__file__).resolve().parent;R=A.parents[3]
original_cover=ForecastGuide._wall_covers_guide
original_fit=entry.tracked_straight_wall
source=R/'src/arena_autonomy/arena_autonomy/bypass_wall_fit.py'
scope=types.ModuleType('geometry_only_fit_candidate')
code=source.read_text(encoding='utf8')
assert code.count('narrow_occluders(points,gaps>limits)')==1
exec(compile(code.replace('narrow_occluders(points,gaps>limits)',
    'narrow_occluders(points,gaps>limits,max_points=len(points))'),'geometry_candidate','exec'),scope.__dict__)

def forward_cover(self,fit,wall):
    if original_cover(self,fit,wall):return True
    if (self.rear_wall_witness_enabled and self.config.narrow_occluder_bridge
            and self.phase=='rejoin' and self.path is not None and self.wall_confirmations>=2
            and wall.get('reason')=='current_connected_straight_wall_fit'
            and wall.get('actual_return_count',0)>=12
            and fit[2]<=self.config.length_m/2 and fit[3]>=.65):
        wall['guide_extent']='current_front_body_and_forward_rejoin_only'
        return True
    return False

@contextmanager
def mode(name):
    ForecastGuide._wall_covers_guide=forward_cover if 'forward' in name else original_cover
    entry.tracked_straight_wall=scope.tracked_straight_wall if 'geometry' in name else original_fit
    try:yield
    finally:
        ForecastGuide._wall_covers_guide=original_cover;entry.tracked_straight_wall=original_fit

dest=A/'c4_extent_probe';dest.mkdir(exist_ok=False)
path=A/'c_stationary_posts/run4/follow_decisions.jsonl'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
hashes={p:sha(p) for p in (path,source)};names=('baseline','geometry','forward','geometry_forward')
controllers={};rows=[];details=Counter();counts=Counter();base_mismatch=0
with (dest/'rows.jsonl').open('w') as out:
    for e in read_events(path):
        d=e['data']
        if e['kind']=='header':
            for name in names:
                with mode(name):controllers[name],mc=build_controller(d)
        elif e['kind']=='reset':
            for name,c in controllers.items():
                with mode(name):c.reset()
        elif e['kind']=='call':
            row=dict(t=d['input']['reference_s'])
            for name,c in controllers.items():
                with mode(name):answer=freeze(replay_call(c,d['input'],mc))
                row[name]=dict(command=answer[:2],reason=answer[2].get('bypass',{}).get('reason'))
                counts[name]+=answer[0]<=1e-6
                if row['t']>=24.52:details[(name,row[name]['reason'])]+=1
                if name=='baseline':base_mismatch+=not equal_output(comparable_decision(answer),comparable_decision(d['output']))
            rows.append(row);out.write(json.dumps(row)+'\n')
summary=dict(calls=len(rows),baseline_mismatches=base_mismatch,zeros=dict(counts),
    after_24_52={name:dict(calls=sum(r['t']>=24.52 for r in rows),
        zeros=sum(r['t']>=24.52 and r[name]['command'][0]<=1e-6 for r in rows),
        reasons={str(reason):n for (variant,reason),n in details.items() if variant==name}) for name in names},
    source_unchanged=all(sha(p)==h for p,h in hashes.items()),
    scope='Saved inputs only; source unchanged; physical cluster extent and forward reference candidates independently varied')
(dest/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary),flush=True)

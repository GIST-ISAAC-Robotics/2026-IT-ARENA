"""기존 복귀 띠와 중앙/중간 편이 경로의 실제 관측을 비교한다. 출력은 원명령이다."""
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
from arena_autonomy.bypass_handoff_corridor_trial import ForecastGuide
import arena_autonomy.bypass_entry_trial as bt

def main():
    out=A/'return_band_probe';out.mkdir(exist_ok=False)
    source=A/'moving_stop_restart/run1/follow_decisions.jsonl'
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in
        [source,*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]}
    original=ForecastGuide.evaluate;rows=[];seen=set();counts=Counter()
    def evaluate(self,scene,history,speed,previous,dt):
        result=original(self,scene,history,speed,previous,dt)
        reference=result.get('reference',{})
        if (not 11.8<=scene.t_ref_s<=14.2 or scene.source_first_s in seen
                or reference.get('reason')!='current_outer_wall_rechecked_for_remembered_guide'
                or self.phase!='entry'):return result
        seen.add(scene.source_first_s);wall=reference['walls'][self.side]
        fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
        band=self._rejoin_band(scene,fit,wall=wall)
        # 현재 함수의 같은 스캔 재평가는 새 관측을 추가하지 않는다.
        padding=self.padding_provider(scene,self.config,self.side,wall)
        row=dict(t=scene.t_ref_s,source=scene.source_first_s,band=band,primary=result['candidates'],alternatives=[])
        for offset in (0.,self.offset*.5,self.offset*.8):
            for transition in self.entry.transitions_m:
                path=bt.entry_path(fit,offset,self.config,self.entry,transition)
                alternative=bt._compare_entries(scene,scene.t_ref_s,speed,previous,dt,
                    config=self.config,entry=self.entry,padding_provider=lambda *args:padding,
                    guide=dict(reference=reference,path=path,offset_m=offset,transition_m=transition,
                        outer_side=self.side,stationary_outer_boundary=self.stationary_outer_boundary and self.wall_confirmations>=2))
                row['alternatives'].append(alternative)
        rows.append(freeze(row));return result
    ForecastGuide.evaluate=evaluate
    try:
        for event in read_events(source):
            data=event['data']
            if event['kind']=='header':controller,motion=build_controller(data)
            elif event['kind']=='reset':controller.reset()
            elif event['kind']=='call':
                result=freeze(replay_call(controller,data['input'],motion));counts['calls']+=1
                counts['command_mismatch']+=not equal_output(result[:2],data['output'][:2])
    finally:ForecastGuide.evaluate=original
    result=dict(counts=dict(counts),rows=rows,sources=sources,
        source_unchanged=all(sha(ROOT/p)==h for p,h in sources.items()))
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(counts=counts,source_unchanged=result['source_unchanged'],rows=[dict(t=r['t'],
        band=r['band'],primary_preview=r['primary'][0].get('preview_return_progress_m'),
        eligible=[{k:c.get(k) for k in ('offset_m','transition_m','target_speed_mps','preview_return_progress_m')}
            for a in r['alternatives'] for c in a['candidates'] if c['eligible']]) for r in rows])))
    assert counts['command_mismatch']==0 and result['source_unchanged']

if __name__=='__main__':main()

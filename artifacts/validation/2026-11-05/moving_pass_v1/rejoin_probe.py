"""오른쪽 12 cm 장면의 실제 복귀 실패: 선택형 안내기가 최종 사용한 현재 벽 기하."""
from pathlib import Path
import json
from arena_autonomy.moving_pass import MovingPassGuide
from arena_autonomy.follow_trace import replay_events
from review import lines,read
HERE=Path(__file__).resolve().parent
if __name__=='__main__':
    d=HERE/'closed_loop_side_pass_1';tg=read(d/'run1/report.json')['case_window']['green_sim_s']
    original=MovingPassGuide._outer_wall_fit;rows=[];seen=set()
    def observe(self,scene,old):
        out=original(self,scene,old);t=scene.t_ref_s-tg
        if self.phase=='rejoin' and (scene.source_first_s not in seen or 8.85<=t<=9.27):
            fit,wall=out;seen.add(scene.source_first_s)
            rows.append(dict(t=round(t,3),fit=fit,ray_count=wall.get('actual_return_count'),
                witness=wall.get('rear_geometry_witness'),reason=wall.get('reason'),
                indices=(wall.get('source_ray_indices') or [])[:1]+(wall.get('source_ray_indices') or [])[-1:],
                tracker=self.tracker.detail.get('reason'),rear=self.tracker.detail.get('rear_clear')))
        return out
    MovingPassGuide._outer_wall_fit=observe
    try:r=replay_events(lines(d/'run1/follow_decisions.jsonl'))
    finally:MovingPassGuide._outer_wall_fit=original
    out=dict(calls=r['calls'],original_mismatches=len(r['mismatches']),rows=rows)
    with (d/'rejoin_reference.json').open('x',encoding='utf-8') as f:json.dump(out,f,indent=1);f.write('\n')
    print('calls/mismatches',r['calls'],len(r['mismatches']))
    print([(s['t'],s['fit']) for s in rows if 9.175<=s['t']<=9.421])

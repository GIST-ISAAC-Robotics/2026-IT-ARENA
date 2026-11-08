"""복귀 벽 확인 실패의 경로 범위를 계측한다. 명령은 바꾸지 않는 저장 입력 분석."""
import json
from pathlib import Path
from arena_autonomy.follow_trace import replay_events
from arena_autonomy.moving_pass import MovingPassGuide

HERE=Path(__file__).resolve().parent
rows=[]
original=MovingPassGuide._wall_covers_guide


def observe(self,fit,wall):
    accepted=original(self,fit,wall)
    rows.append(dict(t=self.reference_s,phase=self.phase,returning_behind=self.returning_behind,
        accepted=accepted,fit=list(fit),wall_reason=wall.get('reason'),count=wall.get('actual_return_count'),
        rear_witness=bool(wall.get('rear_geometry_witness')),front_clear=self.tracker.detail.get('ahead_clear'),
        rear_clear=self.tracker.detail.get('rear_clear'),guide_extent=wall.get('guide_extent')))
    return accepted


MovingPassGuide._wall_covers_guide=observe
with (HERE/'straight_moving_lead/run2/follow_decisions.jsonl').open() as f:
    result=replay_events(json.loads(s) for s in f)
MovingPassGuide._wall_covers_guide=original
(HERE/'guide_extent_rows.json').write_text(json.dumps(rows,indent=1),encoding='utf-8')
bad=[r for r in rows if not r['accepted']]
summary=dict(replay={k:result[k] for k in ('calls','matched')},wall_checks=len(rows),rejected=len(bad),
    first_rejected=bad[0] if bad else None,last_rejected=bad[-1] if bad else None,
    lower_range=[min(r['fit'][2] for r in bad),max(r['fit'][2] for r in bad)] if bad else None,
    upper_range=[min(r['fit'][3] for r in bad),max(r['fit'][3] for r in bad)] if bad else None)
(HERE/'guide_extent_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary))

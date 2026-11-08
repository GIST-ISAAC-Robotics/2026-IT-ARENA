"""복귀 안내의 앞쪽 시작점 대조. 현재 코드 파일을 바꾸지 않는 저장 입력 시험."""
import json
from pathlib import Path
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.moving_pass import MovingPassGuide

HERE=Path(__file__).resolve().parent
SOURCE=HERE.parent/'traffic_surface_alignment_v1/straight_moving_lead/run2/follow_decisions.jsonl'
original=MovingPassGuide._wall_covers_guide


def candidate(self,fit,wall):
    if original(self,fit,wall):return True
    # 이전 조건이 실패한 경우 중 현재 차체의 앞부분 안에서 시작하는 관측만 비교.
    clear=self.tracker.detail.get('ahead_clear') if self.returning_behind else self.tracker.detail.get('rear_clear')
    if (self.phase=='rejoin' and clear and self.tracker.current is not None
            and wall.get('reason')=='current_connected_straight_wall_fit'
            and wall.get('actual_return_count',0)>=12
            and 0.<fit[2]<=self.config.length_m/2 and fit[3]>=.65):
        wall['guide_extent']='current_front_body_and_forward_rejoin_only'
        return True
    return False


events=[json.loads(line) for line in SOURCE.read_text().splitlines()]
header=events[0]['data']
follow=BypassHandoffFollow(FollowConfig(**header['follow_config']),
    HandoffExecutionConfig.from_mapping(header['execution_config']),allow_simulation_trial=True)
motion=MotionConfig(**header['motion_config'])
MovingPassGuide._wall_covers_guide=candidate
rows=[]
for e in events[1:]:
    if e['kind']=='reset':follow.reset()
    elif e['kind']=='call':
        q=e['data'];out=replay_call(follow,q['input'],motion)
        rows.append(dict(t=q['input']['reference_s'],old_speed=q['output'][0],speed=out[0],
            old_steer=q['output'][1],steer=out[1],reason=out[2].get('reason')))
MovingPassGuide._wall_covers_guide=original
summary=dict(calls=len(rows),before1406_changes=sum(
    abs(r['speed']-r['old_speed'])>1e-9 or abs(r['steer']-r['old_steer'])>1e-9 for r in rows if r['t']<14.06),
    new_positive=sum(r['old_speed']==0 and r['speed']>0 for r in rows if r['t']<=17.6),
    first_new_positive=next((r for r in rows if r['old_speed']==0 and r['speed']>0),None),last=rows[-1],
    scope='saved frozen trajectory; not actual recovery or return completion')
(HERE/'shadow_rows.json').write_text(json.dumps(rows,indent=1),encoding='utf-8')
(HERE/'shadow_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary))

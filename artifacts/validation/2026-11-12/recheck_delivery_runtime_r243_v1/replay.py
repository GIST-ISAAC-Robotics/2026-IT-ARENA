"""새 실행 기록의 원 계획/현재 재검사를 분리 재생한다. 대기/발행은 재현하지 않는다."""
import json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import replay_events,equal_output
from arena_autonomy.recheck_process import evaluate_recheck
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig


def main():
    path=A/'two_actor_run1/run1/follow_decisions.jsonl'
    def events():
        with path.open(encoding='utf8') as f:
            for line in f:yield json.loads(line)
    result=replay_events(events())
    result.pop('outcomes')
    header=next(events())['data'];motion=MotionConfig(**header['motion_config']);cfg=FollowConfig(**header['follow_config'])
    checked=0;different=[]
    for event in events():
        if event['kind']!='call' or 'current_rebase' not in event['data']:continue
        call=event['data'];current=call['current_rebase']
        actual=evaluate_recheck(call['output'],call['input'],current['input'],motion,cfg)
        checked+=1
        if not equal_output(actual,current['output']):different.append(dict(seq=event['seq'],actual=actual,expected=current['output']))
    result.update(current_checked=checked,current_differences=different,scope='planner and current recheck separately replayed; runtime timing not replayed')
    (A/'replay_report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('mismatches','current_differences','audit_metadata')}))
    return 0 if result['passed'] and not different else 1


if __name__=='__main__':raise SystemExit(main())

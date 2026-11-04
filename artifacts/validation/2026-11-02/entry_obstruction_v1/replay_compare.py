"""정상 우회 전체 입력과 실패 입력의 첫 분기까지 대조. 분기 후 반사실 주행 주장 금지."""
from pathlib import Path
import json
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import replay_call, comparable_decision, equal_output, freeze
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig


def strip_diagnostic(value):
    if isinstance(value,dict):
        return {k:strip_diagnostic(v) for k,v in value.items() if k!='entry_path_conflict'}
    if isinstance(value,list):
        return [strip_diagnostic(v) for v in value]
    return value


def compare(relative):
    path=HERE.parent/relative/'follow_decisions.jsonl'
    count=0;footer=None;extra=0;first_detail_difference=None
    for sequence,line in enumerate(path.open(encoding='utf-8')):
        event=json.loads(line);assert event['seq']==sequence
        data=event['data']
        if event['kind']=='header':
            controller=BypassHandoffFollow(FollowConfig(**data['follow_config']),
                HandoffExecutionConfig.from_mapping(data['execution_config']),allow_simulation_trial=True)
            motion=MotionConfig(**data['motion_config'])
        elif event['kind']=='reset':
            controller.reset()
        elif event['kind']=='call':
            actual=freeze(replay_call(controller,data['input'],motion));expected=data['output']
            count+=1
            if not equal_output(comparable_decision(strip_diagnostic(actual)),comparable_decision(expected)):
                if first_detail_difference is None:
                    first_detail_difference=dict(call=count,reference_s=data['input']['reference_s'])
                controls_equal=equal_output(actual[:2],expected[:2]) and actual[2].get('handoff_state')==expected[2].get('handoff_state')
                if controls_equal:
                    continue
                return dict(path=relative,control_matched_before_divergence=count-1,first_detail_difference=first_detail_difference,divergence_s=data['input']['reference_s'],
                    expected=dict(speed=expected[0],steering=expected[1],mode=expected[2].get('handoff_state')),
                    actual=dict(speed=actual[0],steering=actual[1],mode=actual[2].get('handoff_state')),
                    admission=actual[2].get('entry_admission'),full_match=False)
            extra+=int('entry_path_conflict' in str(actual[2].get('entry_admission')))
        elif event['kind']=='footer':
            footer=data
    assert footer is not None and footer['capture_errors']==footer['failed']==footer['dropped']==footer['pending']==0
    assert footer['worker_alive'] is False and footer['written']==footer['accepted']==sequence
    return dict(path=relative,calls=count,full_match=first_detail_difference is None,first_detail_difference=first_detail_difference,new_diagnostic_calls=extra)


if __name__=='__main__':
    results=[compare(path) for path in ['official_traffic_v1/p1','official_traffic_v1/r1','official_race_distance_v1/run1']]
    passed=all(r['full_match'] for r in results[:2]) and not results[2]['full_match'] and results[2]['actual']['mode']=='FOLLOW' and results[2]['expected']['mode']=='BYPASS'
    report=dict(passed=passed,results=results,scope='실패 입력은 최초 명령/모드 차이까지만. 이전 진입 후보 진단 차이는 별도로 기록. 이후 실제 주행 결과는 새 폐루프 시험 필요.')
    with (HERE/'replay_control_comparison.json').open('x',encoding='utf-8') as out:
        json.dump(report,out,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(report,ensure_ascii=False))
    raise SystemExit(0 if passed else 1)

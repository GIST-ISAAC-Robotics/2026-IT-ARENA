"""2 m/s 완주 당시 센서를 입력으로 10 km/h 설정이 실제로 요청할 속도를 확인한다."""
from dataclasses import replace
from pathlib import Path
import argparse
import json
import math
import hashlib

from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.local_path import braking_speed


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--diagnostics',action='store_true')
    diagnostics=parser.parse_args().diagnostics
    base=Path(__file__).resolve().parent
    trace=base/'solo_2p0/follow_decisions.jsonl'
    cap=10/3.6
    nr=additional=at_limit=old_zero_to_motion=steer_changes=0
    peak=0.; longest=0.; first=None; last_at=None
    top={}; zero_changes=[]; steer_above_one_degree=0; max_steer_change=0.
    digest=hashlib.sha256(); footer=None
    with trace.open('rb') as stream:
        for seq,line in enumerate(stream):
            digest.update(line); event=json.loads(line); d=event['data']
            assert event['seq']==seq and footer is None
            if event['kind']=='header':
                source=FollowConfig(**d['follow_config'])
                assert source.open_route_speed_mps==2 and source.blend_open_speed_by_length
                controller=LocalFollow(replace(source,open_route_speed_mps=cap))
                motion=MotionConfig(**d['motion_config'])
            elif event['kind']=='reset':
                controller.reset();last_at=None
            elif event['kind']=='call':
                t=d['input']['reference_s'];v,angle,detail=replay_call(controller,d['input'],motion)
                before=d['output']
                nr+=1;additional+=v>2+1e-8;old_zero_to_motion+=before[0]==0 and v>0
                steer_changes+=abs(before[1]-angle)>1e-8
                da=math.degrees(abs(before[1]-angle))
                steer_above_one_degree+=da>1.;max_steer_change=max(max_steer_change,da)
                if diagnostics and v>peak:
                    top={k:detail.get(k) for k in ('selector_choice','path_wall','observed_search_horizon_m',
                        'observed_path_ahead_m','swept_clearance_m','observed_length_speed_cap_mps','source_first_s',
                        'boundary_age_s','reason')}
                    top.update(reference_s=t,command_mps=v,steering_rad=angle)
                    if top['source_first_s'] is not None:
                        age=t-top['source_first_s']
                        if top['observed_path_ahead_m'] is not None:
                            top['path_brake_cap_mps']=braking_speed(top['observed_path_ahead_m'],max(age,top['boundary_age_s'] or 0.))
                        if top['swept_clearance_m'] is not None:
                            free=max(0.,top['swept_clearance_m']-source.standstill_gap_m)
                            top['swept_brake_cap_mps']=braking_speed(free,age)
                            top['swept_headway_cap_mps']=free/source.headway_s
                if diagnostics and before[0]==0 and v>0:
                    zero_changes.append(dict(reference_s=t,command_mps=v,old_reason=before[2].get('reason'),
                                             reason=detail.get('reason'),steering_change_deg=da))
                peak=max(peak,v)
                if math.isclose(v,cap,abs_tol=1e-7):
                    at_limit+=1
                    if last_at is None or t-last_at>.1:first=t
                    last_at=t
                    longest=max(longest,t-first)
                else:last_at=None
            elif event['kind']=='footer':footer=d
    assert footer and all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending')) and footer['written']==seq
    result=dict(trace_sha256=digest.hexdigest(),calls=nr,target_mps=cap,peak_request_mps=peak,
                above_2mps=additional,at_target=at_limit,longest_exact_target_run_s=longest,
                previous_zero_to_motion=old_zero_to_motion,steer_changes=steer_changes,
                scope='동일 저장 궤적에 다른 순항 상한을 적용한 명령 계산. 차체 속도·주행 결과 아님.')
    if diagnostics:
        result.update(highest_request=top,changed_zero_rows=zero_changes,
                      steering_changes_above_one_degree=steer_above_one_degree,max_steering_change_deg=max_steer_change)
    with (base/('ten_kmh_diagnostics.json' if diagnostics else 'ten_kmh_preview.json')).open('x',encoding='utf-8') as out:
        json.dump(result,out,indent=2,ensure_ascii=False)
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()

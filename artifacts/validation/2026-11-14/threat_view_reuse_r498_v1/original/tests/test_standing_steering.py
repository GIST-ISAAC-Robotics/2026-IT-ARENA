"""실제 선두칸 정지 입력에서 정지 조향과 전진 허가를 분리한다."""
import copy
import json
import math
from pathlib import Path

import pytest
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.lidar_motion import MotionConfig

ROOT=Path(__file__).resolve().parents[1]
RAW=ROOT/'artifacts/validation/2026-11-02/official_camera_mount_v1/run1/follow_decisions.jsonl'
pytestmark=pytest.mark.skipif(not RAW.exists(),reason='원시 시뮬 입력은 Git에서 제외된다')


def recorded():
    header=None; calls=[]
    for line in RAW.open():
        event=json.loads(line)
        if event['kind']=='header':header=event['data']
        if event['kind']=='call':
            data=event['data']['input']
            if 19.9 <= data['reference_s'] <= 21.:
                calls.append(data)
    assert header and len(calls)>30
    return header,calls


def replay(enabled, *, measured=0., blocked=False, stale=False):
    header,calls=recorded()
    config={**header['follow_config'],'standing_wall_reacquire':enabled}
    controller=LocalFollow(FollowConfig(**config))
    motion=MotionConfig(**header['motion_config'])
    previous=0.; result=[]
    for raw in calls:
        data=copy.deepcopy(raw)
        data['speed_mps']=measured
        data['previous_steering_rad']=previous
        if stale:data['reference_s']+=1.
        if blocked:
            scan=data['scan']
            for i in range(len(scan['ranges'])):
                angle=scan['angle_min']+i*scan['angle_increment']
                if abs(angle)<.5:
                    scan['ranges'][i]=.10/math.cos(angle)
        output=replay_call(controller,data,motion)
        result.append(output)
        previous=output[1]
        # 저장 입력이 다른 이동으로 바뀌지는 않는다. 최초 전진 허가에서 중단한다.
        if output[0]>0:break
    return result


def test_off_preserves_straight_hold():
    rows=replay(False)
    assert all(v==s==0 for v,s,_ in rows)
    assert any(d['reason']=='follow_gap_hold' for _,_,d in rows)


def test_current_path_allows_steering_then_existing_fresh_release():
    rows=replay(True)
    steering=[r for r in rows if r[2]['reason']=='follow_steer_in_place']
    assert steering and all(r[0]==0 and r[1]<0 for r in steering)
    assert all(not r[2]['boundary_from_memory'] for r in steering)
    assert rows[-1][0]>0 and rows[-1][2]['reason']=='local_follow'
    assert len({r[2].get('source_first_s') for r in rows})>=3
    assert all(abs(b[1]-a[1])<=.0600001 for a,b in zip(rows,rows[1:]))


@pytest.mark.parametrize('speed',[.101,.2,-.2])
def test_nonstationary_measurement_keeps_hold(speed):
    assert all(v==0 and d['reason']!='follow_steer_in_place' for v,_,d in replay(True,measured=speed))


def test_real_front_occupancy_does_not_gain_forward_permission():
    assert all(v==0 for v,_,_ in replay(True,blocked=True))


def test_stale_scene_does_not_steer_or_move():
    assert all(v==s==0 for v,s,_ in replay(True,stale=True))

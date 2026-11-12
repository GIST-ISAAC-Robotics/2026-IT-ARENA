"""실제 저장 현재 입력의 저속 허용과 넓은 공백/점유/속도/선택 경계."""
import copy,json,math,sys
from dataclasses import replace
from pathlib import Path
import pytest
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path.insert(0,str(A.parent/'retained_route_execution_r266_v1'))
from arena_autonomy.follow_trace import restore_call,freeze,thaw,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.numeric_kernels import configure
from overlap import plan_reference
import route_proof
import candidate


@pytest.fixture
def recorded():
    with (A.parent/'opposite_reacquire_runtime_r279_v1/two_actor_run1/run1/follow_decisions.jsonl').open() as f:
        header=json.loads(next(f))['data']
    configure('native_v1')
    return json.loads((A/'example.json').read_bytes()),FollowConfig(**header['follow_config']),MotionConfig(**header['motion_config'])


def evaluate(data,*,speed=None,enabled=True,change=None):
    c,cfg,motion=data;c=copy.deepcopy(c);current=c['current_rebase']['input']
    if not enabled:cfg=replace(cfg,released_body_gap_trial=False)
    if speed is not None:current['speed_mps']=speed
    if change is not None:change(current,motion)
    d,scan,h=restore_call(current,motion)
    scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    args=(c['output'],plan_reference(c['input']),current,scene,h,cfg)
    old=route_proof.rebase(*args,require_current_region=True)
    new=candidate.rebase(*args,require_current_region=True)
    return old,new


def test_current_low_gap_rechecked_without_erasing_unknown(recorded):
    old,new=evaluate(recorded)
    assert old['speed_mps']==0 and new['speed_mps']==.15
    proof=new['current_route_proof']
    assert proof['bounded_current_risk']['accepted'] and proof['region']['unobserved']==332
    assert not proof['unknown_filled'] and not proof['old_occupancy_permission_reused']
    for k in ('reference_s','source_first_s','path_source_s','original_reference_s','steering_rad'):
        assert old[k]==new[k]


@pytest.mark.parametrize('speed',[.35001,.4,1.])
def test_faster_actual_motion_does_not_inherit_low_speed_permission(recorded,speed):
    old,new=evaluate(recorded,speed=speed)
    assert new['speed_mps']==0 and equal_output(freeze(old),freeze(new))


def test_disabled_policy_preserves_zero(recorded):
    old,new=evaluate(recorded,enabled=False)
    assert new['speed_mps']==0 and equal_output(freeze(old),freeze(new))


def test_wider_current_nonreturn_does_not_use_previous_bounded_permission(recorded):
    def widen(current,_):
        m=thaw(current['scan']['measurements'])
        for i in range(90,125):m['ranges_m'][i]=float('inf')
        current['scan']['measurements']=freeze(m)
    old,new=evaluate(recorded,change=widen)
    assert new['speed_mps']==0


def test_current_front_obstacle_blocks_before_gap_exception(recorded):
    def obstacle(current,motion):
        d,scan,h=restore_call(current,motion)
        scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        m=thaw(current['scan']['measurements']);count=0
        for ray in scene.rays:
            if abs(math.atan2(ray.direction_xy[1],ray.direction_xy[0]))<.4:
                m['ranges_m'][ray.source_index]=.10;count+=1
        assert count>5
        current['scan']['measurements']=freeze(m)
    old,new=evaluate(recorded,change=obstacle)
    assert new['speed_mps']==0 and equal_output(freeze(old),freeze(new))

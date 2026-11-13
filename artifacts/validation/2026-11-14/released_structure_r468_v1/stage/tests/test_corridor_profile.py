from arena_autonomy.follow_trace import snapshot_call
from arena_autonomy.corridor_observer import CorridorObserver
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from test_local_follow import scan,history
from profile_corridor_observer import timed_pair
from profile_follow_trace import GCObservation


def test_timing_boundaries_separate_restore_projection_and_two_decisions():
    motion=MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06)
    raw=snapshot_call(scan(first=.5),history(),.6,0,0.,'left',0.,.02,True,30.)
    clock=iter((0,1_000_000,3_000_000,6_000_000,10_000_000))
    baseline,proposal,cost=timed_pair(LocalFollow(),CorridorObserver(FollowConfig()),raw,motion,lambda:next(clock))
    assert cost==dict(restore_ms=1.,projection_ms=2.,baseline_ms=3.,observer_ms=4.,decision_pair_ms=7.,total_ms=10.)
    assert baseline[0]==0 and not proposal['positive']


def test_gc_costs_are_observations_not_subtracted_from_wall_time():
    motion=MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06)
    raw=snapshot_call(scan(first=.5),history(),.6,0,0.,'left',0.,.02,True,30.)
    observation=GCObservation()
    count=iter(range(5))
    def clock():
        stage=next(count)
        if stage==2:
            observation.elapsed_ns=500_000
            observation.collections=1
        return stage*1_000_000
    _,_,cost=timed_pair(LocalFollow(),CorridorObserver(FollowConfig()),raw,motion,clock,observation)
    assert cost['total_ms']==4 and cost['gc_ms']==.5
    assert cost['gc_by_stage_ms']==dict(restore=0.,projection=.5,baseline=0.,observer=0.)
    assert cost['gc_collections']==1 and cost['thread_cpu_ms']>=0

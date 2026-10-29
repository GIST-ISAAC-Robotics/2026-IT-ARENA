"""보존된 진단 반례를 실행용 구현에도 그대로 적용한다. 진단 소스는 수정하지 않는다."""
def pytest_configure(config):
    import probe_timed_policy, probe_timed_missing_sweep, probe_joint_missing_sweep
    import probe_surface_anchor, probe_same_path_gap, recovery_policy
    from arena_autonomy.traffic_recovery_trial import TimedPolicy,RecoveryPolicy
    from arena_autonomy import recovery_geometry as geometry
    from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial
    from arena_autonomy.same_path_gap_trial import bounded_same_path_gap
    probe_timed_policy.policy_class=lambda:(TimedPolicy,'','')
    probe_timed_missing_sweep.timed_missing_sweep=geometry.timed_missing_sweep
    for name in ('joint_missing_sweep','sweep_rectangles','clip_polygon','polygon_distance'):
        setattr(probe_joint_missing_sweep,name,getattr(geometry,name))
    import sys
    from arena_autonomy import surface_anchor_trial
    surface_anchor_trial.SurfaceAnchorProbe=SurfaceAnchorTrial
    sys.modules["probe_surface_anchor"]=surface_anchor_trial
    probe_same_path_gap.candidate=lambda:(bounded_same_path_gap,'')
    recovery_policy.recovery_policy_class=lambda:RecoveryPolicy

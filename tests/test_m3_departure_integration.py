"""출발 후보를 명시한 시뮬 노드만 받으며 보호 경계·기록/재생을 보존한다."""
import json
from pathlib import Path
import pytest
from arena_autonomy.follow_trace import FollowTrace, replay_events
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import ExecutionConfig, M3ExecutionFollow
from test_follow_trace import context, controller
from test_race_session_node import make_node, close, topics

OPTIONS=('rounded_occupancy','straight_with_threat','defer_center_on_straight')


@pytest.mark.parametrize('enabled',[(),('rounded_occupancy',),OPTIONS])
def test_actual_ros_parameters_are_explicit_and_never_publish_safe(tmp_path,enabled):
    node=make_node(g1_follow_enabled='true',g1_m3_execution_trial='true',
                   g1_m3_command_fallback='true',g1_m3_release_to_baseline='true',
                   g1_occupancy_timing_mode='per_ray_candidate',g1_trace_path=str(tmp_path/'trace.jsonl'),
                   **{'g1_m3_'+k:'true' for k in enabled})
    try:
        for k in OPTIONS: assert getattr(node.follow.execution,k)==(k in enabled)
        subscriptions,publishers=topics(node)
        assert '/drive' in publishers and '/drive/safe' not in publishers
        assert not any('eval_' in p or 'pose' in p for p in subscriptions)
        node.control()
        assert node.last_speed==0.
    finally: close(node)


@pytest.mark.parametrize('option',OPTIONS)
def test_options_cannot_enable_nontrial_node(option):
    with pytest.raises(ValueError,match='M3'):
        make_node(**{'g1_m3_'+option:'true'})


@pytest.mark.parametrize('override',[
    dict(g1_m3_command_fallback='false'),dict(g1_m3_release_to_baseline='false'),
    dict(use_sim_time='false'),dict(g1_occupancy_timing_mode='oldest')])
def test_ros_combination_cannot_bypass_trial_prerequisites(tmp_path,override):
    params=dict(g1_follow_enabled='true',g1_m3_execution_trial='true',g1_m3_command_fallback='true',
        g1_m3_release_to_baseline='true',g1_occupancy_timing_mode='per_ray_candidate',
        g1_trace_path=str(tmp_path/'trace.jsonl'),**{'g1_m3_'+k:'true' for k in OPTIONS})
    params.update(override)
    with pytest.raises(ValueError):make_node(**params)


def test_trace_preserves_options_and_replays_fault_and_reset(tmp_path):
    cfg=FollowConfig(occupancy_timing_mode='per_ray_candidate')
    execution=ExecutionConfig(command_fallback=True,release_to_baseline=True,
                               **{k:True for k in OPTIONS})
    path=tmp_path/'trace.jsonl'
    recorder=FollowTrace(path,context()[1].config,cfg,mode='m3_execution_trial',execution_config=execution)
    node=controller(recorder)
    node.follow=M3ExecutionFollow(cfg,execution,allow_simulation_trial=True)
    for ref in (.32,.34,.42,.52):
        node.scan=context(ref)[0];node.motion.last_meta['reference_stamp_s']=ref;node.command(None)
    node.reset_follow('departure_test')
    node.scan.header.frame_id='wrong_frame';assert node.command(None)[0]==0.
    recorder.close()
    events=[json.loads(line) for line in path.read_text().splitlines()]
    assert all(events[0]['data']['execution_config'][k] for k in OPTIONS)
    result=replay_events(events)
    assert result['passed'] and result['matched']==5


@pytest.mark.parametrize('flags,missing',[
    (['--m3-rounded-occupancy'],'--mode m3'),
    (['--m3-straight-with-threat'],'--mode m3'),
    (['--m3-defer-center-on-straight'],'--mode m3'),
    (['--mode','m3','--m3-straight-with-threat'],'--m3-command-fallback'),
    (['--mode','m3','--m3-defer-center-on-straight'],'--m3-command-fallback'),
    (['--mode','m3','--m3-command-fallback','--m3-defer-center-on-straight'],'--m3-release-to-baseline')])
def test_invalid_cli_stops_before_output_or_process(tmp_path,monkeypatch,capsys,flags,missing):
    import validate_g1_traffic as vt
    monkeypatch.setattr('sys.argv',['validate_g1_traffic.py','--case','stop_restart','--seed','0',
        '--mode','g1','--output',str(tmp_path/'out'),*flags])
    with pytest.raises(SystemExit) as exc:vt.main()
    assert exc.value.code==2 and missing in capsys.readouterr().err
    assert not (tmp_path/'out').exists()


def test_launch_defaults_and_required_sources():
    root=Path(__file__).resolve().parents[1]
    text=(root/'src/arena_bringup/launch/g1_follow.launch.py').read_text()
    for k in OPTIONS:assert "DeclareLaunchArgument('g1_m3_"+k+"', default_value='false'" in text
    import validate_g1_traffic as vt
    for k in ('polygon_batch','rounded_occupancy_candidate'):
        assert 'src/arena_autonomy/arena_autonomy/'+k+'.py' in vt.SOURCE_FILES

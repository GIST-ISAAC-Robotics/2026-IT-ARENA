import pytest
import validate_g1_traffic as harness


@pytest.mark.parametrize('extra', [['--mode','m3'],['--mode','r0'],['--case','slow_lead'],
    ['--lidar-test-profile','config/tests/lidar_uniform30_delay50_jitter20.json']])
def test_m4_rejects_unregistered_scope_before_launch(monkeypatch,tmp_path,extra):
    monkeypatch.setattr('sys.argv',['x','--mode','g1','--case','stop_restart','--seed','0',
        '--record-follow-decisions','--occupancy-timing','per_ray_candidate','--race-operation-test',
        '--output',str(tmp_path),*extra])
    monkeypatch.setattr(harness,'load_test_profile',lambda *_:pytest.fail('must reject before launch'))
    with pytest.raises(SystemExit) as error: harness.main()
    assert error.value.code==2


def test_m4_reaches_profile_boundary_only_for_explicit_scope(monkeypatch,tmp_path):
    monkeypatch.setattr('sys.argv',['x','--mode','g1','--case','stop_restart','--seed','0',
        '--record-follow-decisions','--race-operation-test','--output',str(tmp_path)])
    def reached(*_): raise RuntimeError('validated_without_ROS_launch')
    monkeypatch.setattr(harness,'load_test_profile',reached)
    with pytest.raises(RuntimeError,match='validated_without_ROS_launch'): harness.main()


def test_second_start_is_explicit_recorded_g1_test(monkeypatch, tmp_path):
    monkeypatch.setattr('sys.argv', ['x', '--mode', 'g1', '--case', 'stop_restart', '--seed', '0',
        '--record-follow-decisions', '--race-operation-test', 'new_race_restart', '--output', str(tmp_path)])
    def reached(*_): raise RuntimeError('second_start_explicit_only')
    monkeypatch.setattr(harness, 'load_test_profile', reached)
    with pytest.raises(RuntimeError, match='second_start_explicit_only'): harness.main()

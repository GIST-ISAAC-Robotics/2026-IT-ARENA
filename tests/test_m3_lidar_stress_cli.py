"""명시 M3 센서 주입 옵션을 실제 시뮬 시작 이전 CLI 경계에서 검사한다."""
import pytest
import validate_g1_traffic as harness


@pytest.mark.parametrize('mode', ['g1','m3'])
def test_recorded_g1_and_m3_reach_profile_validation(monkeypatch, tmp_path, mode):
    args=['validate_g1_traffic.py','--case','stop_restart','--seed','0','--mode',mode,
          '--record-follow-decisions','--occupancy-timing','per_ray_candidate',
          '--lidar-test-profile','config/tests/lidar_uniform30_delay50_jitter20.json',
          '--output',str(tmp_path)]
    monkeypatch.setattr('sys.argv',args)
    def reached(path,repo):
        assert path.name=='lidar_uniform30_delay50_jitter20.json'
        raise RuntimeError('reached_profile_validation_without_launch')
    monkeypatch.setattr(harness,'load_test_profile',reached)
    with pytest.raises(RuntimeError,match='reached_profile_validation_without_launch'):
        harness.main()


@pytest.mark.parametrize('mode,record', [('g1',False),('m3',False),('r0',False),('r0plus',True)])
def test_unregistered_modes_or_missing_recording_rejected(monkeypatch,tmp_path,mode,record):
    args=['validate_g1_traffic.py','--case','stop_restart','--seed','0','--mode',mode,
          '--lidar-test-profile','config/tests/lidar_uniform30_delay50_jitter20.json',
          '--output',str(tmp_path)]
    if record:
        args.extend(['--record-follow-decisions','--occupancy-timing','per_ray_candidate'])
    monkeypatch.setattr('sys.argv',args)
    monkeypatch.setattr(harness,'load_test_profile',lambda *_:pytest.fail('must reject before loading profile'))
    with pytest.raises(SystemExit) as failure:
        harness.main()
    assert failure.value.code==2

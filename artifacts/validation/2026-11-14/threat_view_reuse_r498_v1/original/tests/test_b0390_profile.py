"""B0390 신규 선택과 과거 B0183 실행 재현을 구분한다."""
import json
import math
from pathlib import Path
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'


def test_current_camera_and_measured_imu_preserve_mount_and_lidar():
    old = yaml.safe_load((PROFILE.parent/'b0183_c1_jr60p20_sim.yaml').read_text(encoding='utf8'))
    new = yaml.safe_load(PROFILE.read_text(encoding='utf8'))
    assert new['lidar'] == old['lidar']
    # 11/11 후속 IMU 실측 반영. 옛 B0183 재현값208은 보존한다.
    assert old['imu']['rate_hz'] == 208.
    for key in ('model', 'xyz_m', 'gyro_full_scale_dps', 'accel_full_scale_g', 'mass_kg', 'source'):
        assert new['imu'][key] == old['imu'][key]
    assert new['imu']['nominal_rate_hz'] == 208.
    assert new['imu']['rate_hz'] == 199.06850941348748
    assert new['imu']['simulated_gyro_stddev_rad_s'] == .0011
    assert new['imu']['simulated_accel_stddev_mps2'] == .0103
    c = new['camera']
    assert c['model'] == 'Arducam_B0390_IMX219'
    assert (c['width_px'], c['height_px'], c['rate_hz'], c['horizontal_fov_deg']) == (1280,720,60,62.2)
    assert c['xyz_m'] == old['camera']['xyz_m'] and c['rpy_rad'] == old['camera']['rpy_rad']
    vfov = math.degrees(2*math.atan(math.tan(math.radians(c['horizontal_fov_deg'])/2)*c['height_px']/c['width_px']))
    assert 37 < vfov < 38  # 4:3 전체 센서 V48.8을 16:9에 그대로 붙이지 않는다.


def test_explicit_course_and_camera_reach_validation_command():
    proc = subprocess.run([sys.executable, '-B', str(ROOT/'scripts/run_current_solo.py'), '/tmp/b0390_args',
                           '--print-args', '--track','official_v2026_11_06',
                           '--local-sensor-config',str(PROFILE)],check=True,capture_output=True,text=True)
    args = json.loads(proc.stdout)
    assert args[args.index('--track')+1] == 'official_v2026_11_06'
    assert args[args.index('--local-sensor-config')+1] == str(PROFILE.resolve())
    assert args[args.index('--pilot-marker-z')+1] == '0.140'
    assert '--g1-solo-straight-budget' not in args


def test_mutually_exclusive_old_mount_override_is_rejected_before_simulation():
    result = subprocess.run([sys.executable,'-B',str(ROOT/'scripts/run_current_solo.py'),'/tmp/b0390_reject',
                             '--print-args','--previous-jr50','--local-sensor-config',str(PROFILE)],capture_output=True)
    assert result.returncode == 2

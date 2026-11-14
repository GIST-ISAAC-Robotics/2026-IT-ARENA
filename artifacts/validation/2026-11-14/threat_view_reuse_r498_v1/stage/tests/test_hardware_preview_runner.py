"""기존 비구동 경계와 선택 영상/펌웨어 종류의 명령 연결 검증."""
import json
from pathlib import Path
import signal
import subprocess
import sys
from types import SimpleNamespace
import pytest
import probe_g1_hardware_shadow as node
import run_hardware_sensor_preview as runner


def args(**changes):
    return SimpleNamespace(output=Path('/tmp/preview-test'),seconds=20.,imu_port='/dev/imu',
        c1_port='/dev/c1',c1_executable=Path('/tmp/c1-sdk'),imu_source='probe',
        estimated_time=True,camera=changes.pop('camera',False),**changes)


def test_default_probe_has_no_camera_actuator_flash_or_mcu_command():
    a=args();cmd=runner.commands(a)
    assert set(cmd)=={'node','imu','c1'}
    assert '--camera' not in cmd['node'] and '--mcu-status' not in cmd['imu']
    assert '--mcu-status' not in cmd['node']
    assert cmd['imu'][2].endswith('bridge_esp32_imu_preview.py')
    assert '--estimated-time' in cmd['node'] and '--scan-clock' in cmd['c1']
    assert cmd['c1'][cmd['c1'].index('--scan-clock')+1]=='tracking'


def test_selected_camera_runs_existing_vision_on_private_topic():
    a=args(camera=True);cmd=runner.commands(a);ros=node.ros_arguments(a,Path('/repo'))
    assert set(cmd)=={'node','imu','c1','camera'} and '--camera' in cmd['node']
    assert '/camera/color/image_raw:=/hardware_preview/camera/image_raw' in ros
    assert 'vision_execution:=process' in ros and 'motion_scan_time_policy:=host_estimated_v1' in ros
    assert '--publish-hz' not in cmd['camera']


def test_selected_publish_limit_does_not_change_sensor_capture_command():
    cmd=runner.commands(args(camera=True,camera_publish_hz=20.))
    assert cmd['camera'][-2:]==['--publish-hz','20.0']
    assert '--publish-hz' not in cmd['imu'] and '--publish-hz' not in cmd['c1']


@pytest.mark.parametrize('extra',[['--camera-publish-hz','20'],
    ['--camera','--camera-publish-hz','nan'],['--camera','--camera-publish-hz','0']])
def test_invalid_publish_limit_fails_before_device_io(tmp_path,extra):
    output=tmp_path/'absent'
    result=subprocess.run([sys.executable,str(Path(runner.__file__)),
        '--output',str(output),'--imu-port','absent-imu','--c1-port','absent-c1',
        '--c1-executable','absent-sdk','--c1-sha256','0'*64,'--plan-only',*extra],capture_output=True,text=True)
    assert result.returncode==2 and not output.exists()


def test_verified_policy_and_no_camera_stay_default_when_not_selected():
    a=args();a.estimated_time=False
    ros=node.ros_arguments(a,Path('/repo'))
    assert not any('vision_execution' in x or 'motion_scan_time_policy' in x for x in ros)
    assert '/scan/native:=/hardware_preview/c1/native' in ros


def test_sensor_link_selects_one_uart_owner_and_status_only():
    a=args();a.imu_source='sensor-link';cmd=runner.commands(a)
    assert cmd['imu'][2].endswith('bridge_sensor_link_preview.py')
    assert '--mcu-status' in cmd['imu']
    assert '--mcu-status' in cmd['node']
    assert sum(a.imu_port in c for c in cmd.values())==1


def test_plan_only_requires_no_ros_or_device_and_creates_no_output(tmp_path):
    output=tmp_path/'absent'
    result=subprocess.run([sys.executable,str(Path(runner.__file__)),
        '--output',str(output),'--imu-port','absent-imu','--c1-port','absent-c1',
        '--c1-executable','absent-sdk','--c1-sha256','0'*64,'--plan-only'],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    payload=json.loads(result.stdout)
    assert not output.exists() and not payload['drive_output'] and not payload['firmware_written']


class Child:
    def __init__(self,timeout=False):self.actions=[];self.timeout=timeout
    def poll(self):return None
    def send_signal(self,sig):self.actions.append(('signal',sig))
    def wait(self,timeout):
        self.actions.append(('wait',timeout))
        if self.timeout and timeout==10.:raise subprocess.TimeoutExpired('child',timeout)
    def kill(self):self.actions.append(('kill',))


def test_shutdown_first_allows_sensor_cleanup():
    c=Child();forced=[];runner.close_child(c,'imu',forced)
    assert c.actions==[('signal',signal.SIGINT),('wait',10.)] and not forced


def test_unresponsive_owned_child_is_recorded_and_bounded():
    c=Child(True);forced=[];runner.close_child(c,'imu',forced)
    assert forced==['imu'] and c.actions[-2:]==[('kill',),('wait',3.)]

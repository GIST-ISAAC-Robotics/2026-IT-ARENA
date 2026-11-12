import importlib.util
import re
from pathlib import Path
from types import SimpleNamespace

A=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('r165_prefix_test',A/'launch_prefix.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def test_only_controller_and_independent_safety_receive_prefix(tmp_path):
    seen=[]
    fake=SimpleNamespace(Popen=lambda command,**kw:seen.append(command))
    namespace={'subprocess':fake}
    exec('def start(command): return subprocess.Popen(command)',namespace)
    m.instrument_start(namespace['start'],A/'delivery_profile.json',tmp_path/'command.json')(['ros2','launch','--noninteractive','arena_bringup','g1_follow.launch.py'])
    cmd=seen[0];pattern=cmd[cmd.index('--launch-prefix-filter')+1]
    assert re.fullmatch(pattern,'/install/bin/lidar_safety')
    assert re.fullmatch(pattern,'/install/bin/g1_g2_handoff_trial')
    for other in ('virtual_mcu','sim_vehicle','robot_state_publisher','ros_gz_bridge'):
        assert not re.fullmatch(pattern,other)
    assert cmd.count('--launch-prefix')==1 and '--noninteractive' in cmd

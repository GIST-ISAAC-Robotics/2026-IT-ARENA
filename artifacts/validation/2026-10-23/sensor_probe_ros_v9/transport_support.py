"""이번 명시 SHM 16 MiB 시험의 전용 환경/읽기 전용 적용 검사."""
import ctypes
import json
import os
from pathlib import Path

from replay_transport import PROFILE_VARIABLES, validate_active

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def configure():
    for key in PROFILE_VARIABLES:
        os.environ.pop(key, None)
    os.environ.update(RMW_IMPLEMENTATION='rmw_fastrtps_cpp', SKIP_DEFAULT_XML='1',
                      ROS_DOMAIN_ID='201', ROS_LOCALHOST_ONLY='0',
                      ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT',
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(HERE/'profile.xml'))


def inspect(node):
    from rclpy.utilities import get_rmw_implementation_identifier
    assert get_rmw_implementation_identifier() == 'rmw_fastrtps_cpp'
    library = ctypes.CDLL(str(ROOT/'build/sensor_transport_pc_v1/libprobe.so'))
    query = library.arena_dds_transport_json
    query.argtypes, query.restype = [ctypes.c_uint], ctypes.c_char_p
    active = json.loads(query(201))
    with (HERE/(node.get_name()+'_active.json')).open('x') as stream:
        json.dump(active, stream, indent=2)
    validate_active(active, 'shm_16m')

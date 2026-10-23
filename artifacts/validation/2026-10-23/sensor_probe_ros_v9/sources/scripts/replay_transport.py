"""재생 전용 DDS 대조. 기본 실행/OS 설정은 바꾸지 않고 로컬 경로만 허용한다."""
import ctypes
import hashlib
import importlib
import json
import os
from pathlib import Path

MODES = ('system', 'udp_loopback', 'shm_default', 'shm_16m')
PROFILE_VARIABLES = ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
    'FASTDDS_ENVIRONMENT_FILE', 'FASTDDS_BUILTIN_TRANSPORTS', 'SKIP_DEFAULT_XML',
    'RMW_FASTRTPS_USE_QOS_FROM_XML', 'RMW_FASTRTPS_PUBLICATION_MODE',
    'ROS_DISCOVERY_SERVER', 'ROS_STATIC_PEERS', 'ROS_SUPER_CLIENT')


def profile_xml(mode):
    if mode not in MODES[1:]:
        raise ValueError('Only explicit local DDS transports have a generated profile')
    if mode == 'udp_loopback':
        details = '<type>UDPv4</type><interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>'
    else:
        details = '<type>SHM</type>'
        if mode == 'shm_16m':
            details += '<segment_size>16777216</segment_size>'
    return ('<?xml version="1.0" encoding="UTF-8"?>\n'
        '<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">'
        '<transport_descriptors><transport_descriptor><transport_id>arena_local</transport_id>'
        + details + '</transport_descriptor></transport_descriptors>'
        '<participant profile_name="arena_replay_local" is_default_profile="true"><rtps>'
        '<userTransports><transport_id>arena_local</transport_id></userTransports>'
        '<useBuiltinTransports>false</useBuiltinTransports></rtps></participant></profiles>\n')


def configure(mode, library, output):
    """Called before rclpy.init; generated profiles replace—not extend—builtin transports."""
    if mode not in MODES:
        raise ValueError('Unknown DDS mode')
    for key in ('ARENA_REPLAY_DDS_MODE', 'ARENA_REPLAY_DDS_LIBRARY', 'ARENA_REPLAY_DDS_OUTPUT'):
        os.environ.pop(key, None)
    if library is None:
        if mode != 'system':
            raise ValueError('Explicit DDS mode requires the compiled in-process auditor')
        return dict(mode=mode, audited=False)
    if os.environ.get('ROS_DISTRO') != 'humble':
        raise ValueError('Transport comparison currently validated only for Humble')
    library = library.resolve(strict=True)
    for key in PROFILE_VARIABLES:
        os.environ.pop(key, None)
    os.environ.update(RMW_IMPLEMENTATION='rmw_fastrtps_cpp', SKIP_DEFAULT_XML='1',
        ROS_LOCALHOST_ONLY='1' if mode == 'system' else '0',
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        ARENA_REPLAY_DDS_MODE=mode, ARENA_REPLAY_DDS_LIBRARY=str(library),
        ARENA_REPLAY_DDS_OUTPUT=str(output.resolve()))
    result = dict(mode=mode, audited=True, library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
                  localhost_isolation='ROS localhost' if mode == 'system' else 'explicit SHM-only or UDP loopback whitelist')
    if mode != 'system':
        xml = profile_xml(mode)
        path = output/'dds_profile.xml'
        with path.open('x', encoding='utf-8') as stream:
            stream.write(xml)
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(path.resolve())
        result['profile_sha256'] = hashlib.sha256(xml.encode()).hexdigest()
    return result


def validate_active(data, mode):
    participants = data.get('participants', [])
    if data.get('error') or len(participants) != 1:
        raise RuntimeError('Expected exactly one inspectable Fast DDS participant')
    participant = participants[0]
    if participant['builtin']:
        raise RuntimeError('Unrestricted builtin transport is not permitted in replay audit')
    transports = participant['transports']
    expected = {'system': ['SHM', 'UDPv4'], 'udp_loopback': ['UDPv4'],
                'shm_default': ['SHM'], 'shm_16m': ['SHM']}[mode]
    if sorted(t['type'] for t in transports) != expected:
        raise RuntimeError('Active transports do not match the requested test')
    for item in transports:
        if item['type'] == 'UDPv4' and item['whitelist'] != ['127.0.0.1']:
            raise RuntimeError('UDP replay must be restricted to loopback')
        if mode == 'shm_16m' and item['segment_size'] != 16777216:
            raise RuntimeError('SHM size override was not applied')


def audit_current(node_name):
    mode = os.environ.get('ARENA_REPLAY_DDS_MODE')
    if mode is None:
        return None
    from rclpy.utilities import get_rmw_implementation_identifier
    if get_rmw_implementation_identifier() != 'rmw_fastrtps_cpp':
        raise RuntimeError('Transport audit requires Fast DDS RMW')
    library = ctypes.CDLL(os.environ['ARENA_REPLAY_DDS_LIBRARY'])
    query = library.arena_dds_transport_json
    query.argtypes, query.restype = [ctypes.c_uint], ctypes.c_char_p
    data = json.loads(query(int(os.environ['ROS_DOMAIN_ID'])))
    data.update(node=node_name, pid=os.getpid(), mode=mode)
    # Preserve failed inspections too; failed validation prevents sensor playback.
    destination = Path(os.environ['ARENA_REPLAY_DDS_OUTPUT'])/f'dds_active_{node_name}.json'
    with destination.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2)
    validate_active(data, mode)
    return data


def run_controller(name):
    """Only the replay launch wrapper changes; production node lifecycle is unchanged."""
    classes = {'local_pursuit': 'LocalPursuit', 'lidar_safety': 'LidarSafety'}
    cls = getattr(importlib.import_module('arena_autonomy.'+name), classes[name])
    from arena_vehicle_interface.node_lifecycle import run_node
    def factory():
        node = cls()
        try:
            audit_current(name)
        except BaseException:
            node.destroy_node()
            raise
        return node
    run_node(factory)

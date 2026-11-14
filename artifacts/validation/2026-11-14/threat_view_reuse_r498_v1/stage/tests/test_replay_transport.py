"""로컬 전용 DDS 프로필과 적용 확인 실패 경계. 네트워크 없는 검사."""
import importlib.util
import io
import json
import os
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

spec = importlib.util.spec_from_file_location('replay_transport',
    Path(__file__).resolve().parents[1]/'scripts/replay_transport.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def active(transports, builtin=False):
    return {'participants': [{'builtin': builtin, 'transports': transports}]}


UDP = {'type': 'UDPv4', 'whitelist': ['127.0.0.1']}
SHM = {'type': 'SHM', 'segment_size': 524288, 'queue_capacity': 512}


class TransportTest(unittest.TestCase):
    def test_profiles_are_explicit_and_local(self):
        ns = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
        for mode in module.MODES[1:]:
            root = ET.fromstring(module.profile_xml(mode))
            self.assertEqual(root.find('.//d:useBuiltinTransports', ns).text, 'false')
            participant = root.find('d:participant', ns)
            self.assertEqual(participant.attrib['is_default_profile'], 'true')
            transport = root.find('.//d:transport_descriptor', ns)
            if mode == 'udp_loopback':
                self.assertEqual(transport.find('d:type', ns).text, 'UDPv4')
                self.assertEqual([r.text for r in root.findall('.//d:address', ns)], ['127.0.0.1'])
            else:
                self.assertEqual(transport.find('d:type', ns).text, 'SHM')
                self.assertNotIn('UDP', module.profile_xml(mode))
            self.assertIsNone(root.find('.//d:data_writer', ns))  # no sensor QoS override

    def test_profile_rejects_unknown_or_implicit_mode(self):
        for mode in ('bad', 'system'):
            with self.assertRaises(ValueError):
                module.profile_xml(mode)

    def test_known_applied_profiles(self):
        module.validate_active(active([UDP, SHM]), 'system')
        module.validate_active(active([UDP]), 'udp_loopback')
        module.validate_active(active([SHM]), 'shm_default')
        module.validate_active(active([dict(SHM, segment_size=16777216)]), 'shm_16m')

    def test_rejects_ignored_profile_or_wrong_whitelist(self):
        for data in (active([UDP], True), active([UDP, SHM]),
                     active([dict(UDP, whitelist=[])]),
                     active([dict(UDP, whitelist=['192.168.0.134'])]),
                     active([dict(type='unknown')])):
            with self.assertRaises(RuntimeError):
                module.validate_active(data, 'udp_loopback')

    def test_missing_ambiguous_and_wrong_sized_participants(self):
        for data in ({'participants': []}, {'error': 'failed'},
                     {'participants': active([UDP])['participants']*2}):
            with self.assertRaises(RuntimeError):
                module.validate_active(data, 'udp_loopback')
        with self.assertRaises(RuntimeError):
            module.validate_active(active([SHM]), 'shm_16m')

    def test_override_requires_auditor_and_humble(self):
        with self.assertRaises(ValueError):
            module.configure('udp_loopback', None, Path('.'))
        with patch.dict(os.environ, {'ROS_DISTRO': 'jazzy'}, clear=True):
            with self.assertRaises(ValueError):
                module.configure('udp_loopback', Path('library'), Path('.'))

    def test_default_does_not_load_probe_or_reconfigure_dds(self):
        with patch.dict(os.environ, {'RMW_IMPLEMENTATION': 'existing',
                                    'ARENA_REPLAY_DDS_MODE': 'stale'}, clear=True):
            result = module.configure('system', None, Path('.'))
            self.assertEqual(result, {'mode': 'system', 'audited': False})
            self.assertEqual(os.environ['RMW_IMPLEMENTATION'], 'existing')
            self.assertIsNone(module.audit_current('node'))


if __name__ == '__main__':
    unittest.main()

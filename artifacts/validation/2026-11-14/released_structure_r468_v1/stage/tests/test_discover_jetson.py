"""네트워크 전송 없이 발견 응답의 이름/출처/범위 경계를 검사한다."""
import importlib.util
import ipaddress
from pathlib import Path
import socket
import struct
import unittest
from unittest.mock import patch
import sys
import types

spec=importlib.util.spec_from_file_location('discover_jetson',Path(__file__).resolve().parents[1]/'scripts/discover_jetson.py')
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
NET=ipaddress.ip_network('192.168.0.0/24')
NAME='jetson-orin.local'


def packet(name=NAME,address='192.168.0.134',ttl=120,kind=1):
    label=b''.join(bytes([len(s)])+s.encode() for s in name.split('.'))+b'\0'
    return struct.pack('!HHHHHH',0,0x8400,0,1,0,0)+label+struct.pack('!HHIH',kind,0x8001,ttl,4)+socket.inet_aton(address)


class Discovery(unittest.TestCase):
    def parse(self,data,source=('192.168.0.134',5353)):
        return mod.ipv4_answers(data,NAME,NET,source)

    def test_matching_local_a_record(self):
        self.assertEqual(self.parse(packet()),{'192.168.0.134'})

    def test_other_names_and_external_addresses_not_candidates(self):
        for p in (packet(name='another-jetson.local'),packet(address='218.38.137.28'),
                  packet(address='127.0.0.1'),packet(address='192.168.0.255'),packet(ttl=0),packet(kind=28)):
            self.assertEqual(self.parse(p),set())

    def test_response_origin_and_port(self):
        self.assertEqual(self.parse(packet(),('218.38.137.28',5353)),set())
        self.assertEqual(self.parse(packet(),('192.168.0.134',53)),set())

    def test_truncation_and_pointer_cycle(self):
        p=packet()
        for size in range(len(p)):
            self.assertEqual(self.parse(p[:size]),set())
        cycle=struct.pack('!HHHHHH',0,0x8400,0,1,0,0)+b'\xc0\x0c'+b'\0'*14
        self.assertEqual(self.parse(cycle),set())

    def test_compressed_answer(self):
        name=b'\x0bjetson-orin\x05local\0'
        p=struct.pack('!HHHHHH',0x1a11,0x8400,1,1,0,0)+name+struct.pack('!HH',1,1)
        p+=b'\xc0\x0c'+struct.pack('!HHIH',1,1,10,4)+socket.inet_aton('192.168.0.134')
        self.assertEqual(self.parse(p),{'192.168.0.134'})


class IdentityVerification(unittest.TestCase):
    def fake_runtime(self, pinned, offered):
        class Key:
            def get_name(self):return 'ssh-ed25519'
        key=Key()
        expected=key if offered else Key()
        class Transport:
            def __init__(self, connection):self.connection=connection
            def start_client(self, timeout):pass
            def get_remote_server_key(self):return key
            def close(self):self.connection.close()
        class Connection:
            def close(self):pass
        runtime=types.SimpleNamespace(
            HostKeys=lambda _:types.SimpleNamespace(lookup=lambda _: {'ssh-ed25519':expected} if pinned else None),
            Transport=Transport,SSHException=type('SSHException',(Exception,),{}))
        return runtime,Connection

    def test_known_device_key_accepted_without_authentication(self):
        runtime,connection=self.fake_runtime(True,True)
        with patch.dict(sys.modules,{'paramiko':runtime}),patch.object(mod.socket,'create_connection',return_value=connection()):
            verified,rejected=mod.verify(['192.168.0.134'],'isaac-jetson.local',22,'unused')
        self.assertEqual(verified,['192.168.0.134']);self.assertEqual(rejected,[])

    def test_other_device_key_rejected(self):
        runtime,connection=self.fake_runtime(True,False)
        with patch.dict(sys.modules,{'paramiko':runtime}),patch.object(mod.socket,'create_connection',return_value=connection()):
            verified,rejected=mod.verify(['192.168.0.134'],'isaac-jetson.local',22,'unused')
        self.assertEqual(verified,[]);self.assertEqual(rejected[0]['reason'],'host_key_mismatch')

    def test_missing_pinned_identity_does_not_connect(self):
        runtime,_=self.fake_runtime(False,True)
        with patch.dict(sys.modules,{'paramiko':runtime}),patch.object(mod.socket,'create_connection') as connect:
            with self.assertRaisesRegex(ValueError,'no existing pinned SSH identity'):
                mod.verify(['192.168.0.134'],'isaac-jetson.local',22,'unused')
            connect.assert_not_called()


if __name__=='__main__':unittest.main()

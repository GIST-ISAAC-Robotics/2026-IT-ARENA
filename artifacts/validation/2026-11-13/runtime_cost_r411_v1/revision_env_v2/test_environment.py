"""메타데이터 오류만 주입하는 작은 회귀. 제어기/native/원격/장치를 실행하지 않는다."""
import io
import json
import unittest
from unittest.mock import patch
import zipfile
import remote_bootstrap as boot


class FakePath:
    responses={}
    read_sizes=[]
    def __init__(self,name):self.name=name
    def __str__(self):return self.name
    def glob(self,_):return []
    def open(self,mode):
        assert mode=='rb'
        name=self.name
        class Reader:
            def __enter__(self):return self
            def __exit__(self,*_):return False
            def read(self,size):
                FakePath.read_sizes.append((name,size))
                value=FakePath.responses[name]
                if isinstance(value,Exception):raise value
                return value[:size] if isinstance(value,bytes) else value
        return Reader()


class EnvironmentTest(unittest.TestCase):
    def snapshot(self,first):
        FakePath.responses={'/proc/loadavg':first,'/proc/meminfo':b'normal memory'}
        FakePath.read_sizes=[]
        with patch.object(boot,'Path',FakePath):return boot.environment_snapshot()

    def test_oserror_keeps_later_normal_metadata(self):
        result=self.snapshot(OSError('read unavailable'))
        self.assertFalse(result['readings']['/proc/loadavg']['available'])
        self.assertIn('OSError',result['readings']['/proc/loadavg']['error'])
        self.assertEqual(result['readings']['/proc/meminfo'],'normal memory')
        self.assertIn('cpu_count',result)

    def test_typeerror_keeps_later_normal_metadata(self):
        result=self.snapshot(TypeError("can't concat NoneType to bytes"))
        self.assertFalse(result['readings']['/proc/loadavg']['available'])
        self.assertIn('TypeError',result['readings']['/proc/loadavg']['error'])
        self.assertEqual(result['readings']['/proc/meminfo'],'normal memory')

    def test_read_is_bounded_and_none_is_a_per_path_error(self):
        result=self.snapshot(b'x'*5000)
        self.assertEqual(len(result['readings']['/proc/loadavg']),4096)
        self.assertEqual(FakePath.read_sizes,[('/proc/loadavg',4096),('/proc/meminfo',4096)])
        result=self.snapshot(None)
        self.assertIn('TypeError',result['readings']['/proc/loadavg']['error'])
        self.assertEqual(result['readings']['/proc/meminfo'],'normal memory')

    def test_whole_metadata_failure_does_not_block_receipt_or_zip(self):
        for error in (OSError('glob unavailable'),TypeError('metadata unavailable'),RuntimeError('metadata only')):
            with self.subTest(error=type(error).__name__),patch.object(boot,'environment_snapshot',side_effect=error):
                receipt=dict(environment_before=boot.safe_environment_snapshot(),environment_after=boot.safe_environment_snapshot())
                self.assertFalse(receipt['environment_before']['available'])
                data=json.dumps(receipt).encode();buffer=io.BytesIO()
                with zipfile.ZipFile(buffer,'w') as out:out.writestr('execution.json',data)
                with zipfile.ZipFile(buffer) as result:self.assertEqual(result.read('execution.json'),data)


if __name__=='__main__':unittest.main(verbosity=2)

"""ROS 실행 없이 최소 수신자 버퍼·파일 무결성·실패 종료 경계를 검사한다."""
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from replay_imu_probe import ImuProbe, StampBuffer, read_result, TOPIC


def fixture(path):
    buffer = StampBuffer(3)
    buffer.append(4, 10)
    buffer.append(8, 20)
    buffer.save(path/'samples.jsonl')
    result = dict(complete=True, errors=[], count=2,
        samples_sha256=hashlib.sha256((path/'samples.jsonl').read_bytes()).hexdigest(),
        topic=TOPIC, qos=dict(depth=5, reliability='BEST_EFFORT', history='KEEP_LAST'),
        executor_mode='legacy')
    (path/'result.json').write_text(json.dumps(result))
    return result


class ProbeTests(unittest.TestCase):
    def test_buffer_is_bounded_and_keeps_source_order(self):
        b = StampBuffer(2)
        b.append(8, 10)
        b.append(4, 20)
        self.assertEqual(b.rows, [(8, 10), (4, 20)])
        with self.assertRaises(RuntimeError): b.append(12, 30)
        self.assertEqual(len(b.rows), 2)
        with self.assertRaises(ValueError): StampBuffer(0)

    def test_complete_capture_does_not_assert_zero_sensor_loss(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)
            expected = fixture(path)
            result, rows = read_result(path)
            self.assertEqual(result, expected)
            self.assertEqual([r['source_ns'] for r in rows], [4, 8])
            self.assertNotIn('lossless', result)

    def test_existing_samples_are_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'samples.jsonl'
            b = StampBuffer()
            b.save(path)
            with self.assertRaises(FileExistsError): b.save(path)

    def test_incomplete_or_contract_changed_rejected(self):
        variants = [dict(complete=False), dict(errors=['failure']), dict(count=3),
            dict(samples_sha256='wrong'), dict(topic='/imu/data'), dict(executor_mode='retained'),
            dict(qos=dict(depth=100, reliability='RELIABLE', history='KEEP_LAST'))]
        for variant in variants:
            with self.subTest(variant=variant), tempfile.TemporaryDirectory() as temp:
                path = Path(temp)
                result = fixture(path)
                result.update(variant)
                (path/'result.json').write_text(json.dumps(result))
                with self.assertRaises(ValueError): read_result(path)

    def test_samples_tampering_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)
            fixture(path)
            (path/'samples.jsonl').write_text('')
            with self.assertRaises(ValueError): read_result(path)

    def test_callback_clock_reversal_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)
            result = fixture(path)
            data = (path/'samples.jsonl').read_text().replace('20', '0')
            (path/'samples.jsonl').write_text(data)
            result['samples_sha256'] = hashlib.sha256((path/'samples.jsonl').read_bytes()).hexdigest()
            (path/'result.json').write_text(json.dumps(result))
            with self.assertRaises(ValueError): read_result(path)

    def test_early_exit_is_failure_even_when_zero(self):
        p = ImuProbe('unused')
        p.process = Mock(returncode=0)
        p.process.poll.return_value = 0
        with self.assertRaises(RuntimeError): p.check()

    def test_close_sends_interrupt_and_checks_artifact(self):
        p = ImuProbe('unused')
        p.process = Mock(returncode=0)
        p.process.poll.return_value = None
        p.log = Mock()
        with patch('replay_imu_probe.read_result', return_value=({'complete': True}, [])):
            self.assertEqual(p.close(), {'complete': True})
        p.process.send_signal.assert_called_once_with(signal.SIGINT)
        p.log.close.assert_called_once()

    def test_forced_or_nonzero_cleanup_is_not_success(self):
        for timeout in (False, True):
            p = ImuProbe('unused')
            p.process = Mock(returncode=1)
            p.process.poll.return_value = None
            if timeout:
                p.process.wait.side_effect = [subprocess.TimeoutExpired('test', 5), 1]
            with self.assertRaises(RuntimeError): p.close()
            if timeout: p.process.kill.assert_called_once()


if __name__ == '__main__':
    unittest.main()

"""실제 발행식 + 실제 host 값 검사를 연결한다. ROS/센서/송신은 실행하지 않는다."""
import ast
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.vehicle_geometry import float32_limit_not_above
from receiver_owned_actuation_host import ReceiverLeaseHostSession


def expression(file, target):
    tree = ast.parse((HERE/'stage/src/arena_autonomy/arena_autonomy'/file).read_text(encoding='utf8'))
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(target == ast.unparse(t) for t in node.targets):
            value = node.value.elts[0] if isinstance(node.value, ast.Tuple) else node.value
            return compile(ast.Expression(value), str(file), 'eval')
    raise AssertionError('actual publish assignment missing')


class WireBoundary(unittest.TestCase):
    def test_panel_and_single_jog_point_one_never_round_above_host_limit(self):
        # 실제 host.prepare_drive의 숫자 검사를 실행하되 그 뒤의 시각/ARM 단계는 중단한다.
        host = SimpleNamespace(max_speed_mps=.1, max_steering_rad=.5, _sequence=0,
            timing=SimpleNamespace(source_ttl_ns=1, policy_note='file-only test'),
            _result=lambda reason, **kw: dict(reason=reason, **kw),
            review_source=lambda *a, **kw: dict(session_conditions_satisfied=False,
                                               reason='source_unavailable_after_value_gate'))
        wire = lambda x: struct.unpack('<f', struct.pack('<f', x))[0]
        old = ReceiverLeaseHostSession.prepare_drive(host, speed_mps=wire(.1), steering_rad=0., source=None, now=None)
        self.assertEqual(old['reason'], 'invalid_drive_values')
        for filename, target in [('development_jog.py', 'msg.drive.speed'),
                                 ('wall_follow.py', '(message.drive.speed, message.drive.steering_angle)')]:
            expr = expression(filename, target)
            for requested in (0., .05, .1):
                converted = eval(expr, {'float32_limit_not_above':float32_limit_not_above, 'speed':requested})
                self.assertLessEqual(wire(converted), requested)
                if requested > 0: self.assertGreater(wire(converted), 0.)
                result = ReceiverLeaseHostSession.prepare_drive(host, speed_mps=wire(converted),
                            steering_rad=0., source=None, now=None)
                self.assertEqual(result['reason'], 'source_unavailable_after_value_gate')
                self.assertIsNone(result['envelope'])  # ARM/시각 증명이나 명령 생성이 아니다.
            # 음수의 기존 경로는 보존하며 실제 host가 계속 거절한다.
            negative = eval(expr, {'float32_limit_not_above':float32_limit_not_above, 'speed':-.1})
            self.assertEqual(negative, -.1)


if __name__ == '__main__': unittest.main()

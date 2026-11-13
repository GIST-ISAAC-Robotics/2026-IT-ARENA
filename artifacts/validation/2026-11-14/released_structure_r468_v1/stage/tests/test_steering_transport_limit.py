"""실제 포화각 float32 반올림 재현. MCU 검사/물리 한도를 완화하지 않는다."""
import ast
import math
from pathlib import Path
import random
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, ContractConfig

# launch/Gazebo 없이 동일 함수의 수치 경계를 검사한다.
source = ROOT/'src/arena_bringup/launch/simulation.launch.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name=='_float32_limit_not_above')
scope = dict(math=math, struct=struct)
exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), scope)
inward = scope['_float32_limit_not_above']
PHYSICAL = math.atan(.145/(.145/math.tan(.45)+.135/2))


def f32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


def ready_receiver():
    receiver = ActuationReceiver(ContractConfig(max_steering_rad=PHYSICAL), boot_id='test')
    producer = CommandProducer(client_id='test')
    for step in range(23):
        now = step*10_000
        receiver.update_encoder(now, sequence=step, capture_us=now, left_ticks=0, right_ticks=0)
        receiver.tick(now)
    assert receiver.receive(producer.control(receiver.offer(now), 'ARM'), now)[0]
    return receiver, producer, now


class SteeringTransportTests(unittest.TestCase):
    def test_reproduces_old_rounding_failure(self):
        self.assertGreater(f32(PHYSICAL), PHYSICAL)
        r,h,t = ready_receiver()
        frame = h.drive(r.offer(t), speed_mps=1., steering_rad=f32(PHYSICAL), source_us=t, host_now_us=t)
        self.assertEqual(r.receive(frame,t), (False,'invalid_drive'))

    def test_positive_and_negative_inward_bounds_pass_strict_receiver(self):
        for sign in (-1,1):
            r,h,t = ready_receiver()
            value = f32(sign*inward(PHYSICAL))
            self.assertLessEqual(abs(value), PHYSICAL)
            frame = h.drive(r.offer(t), speed_mps=1., steering_rad=value, source_us=t, host_now_us=t)
            self.assertTrue(r.receive(frame,t)[0])

    def test_value_is_largest_float32_inside_limit(self):
        result = inward(PHYSICAL)
        self.assertEqual(f32(result), result)
        self.assertLessEqual(result, PHYSICAL)
        bits = struct.unpack('<I',struct.pack('<f', result))[0]
        self.assertGreater(struct.unpack('<f',struct.pack('<I',bits+1))[0], PHYSICAL)

    def test_exact_and_varied_limits_never_expand(self):
        rng = random.Random(49)
        for value in [.25,.5,1.,2.5]+[rng.uniform(.01,1.4) for _ in range(500)]:
            result = inward(value)
            self.assertGreater(result, 0)
            self.assertLessEqual(f32(result), value)
            self.assertLessEqual(abs(f32(-result)), value)
        self.assertEqual(inward(.5),.5)

    def test_invalid_limits_rejected(self):
        for value in (0,-1,math.nan,math.inf,-math.inf,1e300,1e-100):
            with self.assertRaises(ValueError): inward(value)

    def test_genuine_over_limit_still_faults(self):
        r,h,t = ready_receiver()
        frame = h.drive(r.offer(t), speed_mps=1., steering_rad=PHYSICAL+.001, source_us=t, host_now_us=t)
        self.assertEqual(r.receive(frame,t),(False,'invalid_drive'))

    def test_ros_serialization_stays_inside(self):
        try:
            from ackermann_msgs.msg import AckermannDriveStamped
            from rclpy.serialization import serialize_message, deserialize_message
        except ImportError:
            self.skipTest('ROS environment required for CDR round trip')
        for sign in (-1,1):
            message = AckermannDriveStamped()
            message.drive.steering_angle = sign*inward(PHYSICAL)
            restored = deserialize_message(serialize_message(message), AckermannDriveStamped)
            self.assertLessEqual(abs(restored.drive.steering_angle), PHYSICAL)
            self.assertEqual(restored.drive.steering_angle, message.drive.steering_angle)


if __name__ == '__main__':
    unittest.main()

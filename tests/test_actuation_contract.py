"""ROS/보드 없이 실행할 수 있는 구동 계약 회귀. python tests/test_actuation_contract.py"""
import json
import math
from pathlib import Path
import random
import struct
import sys
import unittest
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/arena_vehicle_interface"))
from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, ContractConfig, State
from arena_vehicle_interface.actuation_wire import Decoder, encode, MAGIC, MAX_PAYLOAD
from arena_vehicle_interface.mcu_speed_loop import SpeedPI


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.r = ActuationReceiver(boot_id="boot-a")
        self.h = CommandProducer(client_id="host-a")
        self.t, self.sample, self.ticks = 0, 0, 0
        self.r.update_encoder(0, sequence=0, capture_us=0, left_ticks=0, right_ticks=0)
        self.advance(220_000)

    def advance(self, duration, *, feedback=True, delta=0):
        for _ in range(duration // 10_000):
            self.t += 10_000
            if feedback:
                self.sample += 1
                self.ticks += delta
                self.r.update_encoder(self.t, sequence=self.sample, capture_us=self.t,
                                      left_ticks=self.ticks, right_ticks=self.ticks)
            self.r.tick(self.t)

    def control(self, kind):
        return self.r.receive(self.h.control(self.r.offer(self.t), kind), self.t)

    def drive(self, speed=1., angle=.2, age=0):
        offer = self.r.offer(self.t)
        packet = self.h.drive(offer, speed_mps=speed, steering_rad=angle,
                              source_us=self.t - age, host_now_us=self.t)
        return packet, self.r.receive(packet, self.t)

    def arm(self):
        self.assertTrue(self.control("ARM")[0])

    def test_boot_no_drive(self):
        self.assertEqual(self.r.state, State.DISARMED)
        with self.assertRaises(ValueError):
            self.drive()
        self.assertEqual(self.r.target_speed, 0)

    def test_arm_needs_settled_fresh_feedback(self):
        r = ActuationReceiver()
        packet = self.h.control(r.offer(0), "ARM")
        self.assertFalse(r.receive(packet, 0)[0])
        self.assertFalse(r.armed)

    def test_normal_zero_then_drive(self):
        self.arm()
        self.assertTrue(self.drive(0)[1][0])
        self.assertEqual(self.r.state, State.ARMED)
        self.advance(10_000)
        self.assertTrue(self.drive()[1][0])
        self.assertEqual(self.r.state, State.ACTIVE)

    def test_duplicate_cannot_extend_lifetime(self):
        self.arm()
        packet, _ = self.drive()
        expiry = self.r.deadline_us
        self.advance(50_000)
        self.assertEqual(self.r.receive(packet, self.t), (False, "sequence"))
        self.assertEqual(self.r.deadline_us, expiry)
        self.advance(50_000)
        self.assertEqual(self.r.reason, "command_expired")
        self.assertEqual(self.r.target_speed, 0)

    def test_old_source_not_repackaged(self):
        self.arm()
        self.drive()
        self.advance(10_000)
        with self.assertRaises(ValueError):
            self.h.drive(self.r.offer(self.t), speed_mps=1., steering_rad=0.,
                         source_us=self.t - 10_000, host_now_us=self.t)

    def test_delay_and_source_age_subtract_from_lease(self):
        self.arm()
        offer = self.r.offer(self.t)
        packet = self.h.drive(offer, speed_mps=1., steering_rad=.1,
                              source_us=self.t - 30_000, host_now_us=self.t)
        self.advance(20_000)
        self.assertTrue(self.r.receive(packet, self.t)[0])
        self.assertEqual(self.r.deadline_us - self.t, 50_000)

    def test_expired_transport_packet(self):
        self.arm()
        packet, _ = self.drive()
        packet["seq"] += 1
        self.advance(100_000)
        self.assertFalse(self.r.receive(packet, self.t)[0])
        self.assertEqual(self.r.state, State.FAULT)

    def test_fresh_lease_cannot_rescue_expired_original_source(self):
        self.arm()
        with self.assertRaises(ValueError):
            self.drive(age=100_000)

    def test_clock_domains_need_not_match(self):
        self.arm()
        packet = self.h.drive(self.r.offer(self.t), speed_mps=1., steering_rad=0.,
                              source_us=9_000_000, host_now_us=9_000_010)
        self.assertTrue(self.r.receive(packet, self.t)[0])
        self.assertEqual(self.r.deadline_us, self.t + 99_990)

    def test_stop_release_clear_and_arm_are_separate(self):
        self.arm()
        old, _ = self.drive()
        self.r.set_estop(True, self.t)
        self.assertEqual(self.r.state, State.ESTOP)
        self.assertFalse(self.control("CLEAR")[0])
        self.r.set_estop(False, self.t)
        self.assertEqual(self.r.state, State.ESTOP)
        self.assertFalse(self.control("ARM")[0])
        self.assertTrue(self.control("CLEAR")[0])
        self.assertEqual(self.r.state, State.DISARMED)
        self.assertFalse(self.r.receive(old, self.t)[0])
        self.arm()
        self.assertEqual(self.r.target_speed, 0)
        self.assertFalse(self.r.receive(old, self.t)[0])

    def test_software_stop_ignores_owner_and_old_lease(self):
        self.arm()
        self.drive()
        self.assertTrue(self.r.receive({"v": 1, "kind": "STOP"}, self.t)[0])
        self.assertEqual(self.r.state, State.ESTOP)
        self.assertEqual(self.r.steering, .2)

    def test_clear_rejected_while_wheels_move(self):
        self.arm()
        self.drive()
        self.advance(10_000, delta=100)
        self.r.set_estop(True, self.t)
        self.r.set_estop(False, self.t)
        self.assertFalse(self.control("CLEAR")[0])
        self.advance(220_000)
        self.assertTrue(self.control("CLEAR")[0])

    def test_opposing_wheel_speeds_not_stationary(self):
        self.t += 10_000
        self.r.update_encoder(self.t, sequence=23, capture_us=self.t, left_ticks=100, right_ticks=-100)
        self.assertEqual(self.r.left_mps + self.r.right_mps, 0)
        self.assertFalse(self.control("ARM")[0])

    def test_reboot_rejects_previous_session(self):
        self.arm()
        old, _ = self.drive()
        r = ActuationReceiver(boot_id="boot-b")
        self.assertFalse(r.receive(old, 0)[0])
        self.assertEqual(r.state, State.DISARMED)

    def test_new_jetson_process_needs_explicit_arm(self):
        self.arm()
        h = CommandProducer(client_id="host-b")
        with self.assertRaises(ValueError):
            h.drive(self.r.offer(self.t), speed_mps=1., steering_rad=0., source_us=0, host_now_us=0)

    def test_invalid_values_fail_closed(self):
        for key, value in [("speed_mps", float("nan")), ("speed_mps", float("inf")),
                           ("speed_mps", -1), ("speed_mps", 2.51), ("speed_mps", True),
                           ("speed_mps", 10**400),
                           ("steering_rad", .38), ("ttl_us", 100001), ("source_age_us", True)]:
            with self.subTest(key=key, value=value):
                self.setUp()
                self.arm()
                packet, _ = self.drive()
                packet.update({key: value, "seq": packet["seq"] + 1})
                self.assertFalse(self.r.receive(packet, self.t)[0])
                self.assertEqual(self.r.target_speed, 0)
                self.assertEqual(self.r.state, State.FAULT)

    def test_schema_and_integer_validation(self):
        self.arm()
        packet, _ = self.drive()
        for bad in [dict(packet, extra=1), dict(packet, seq=True), dict(packet, v=True),
                    dict(packet, generation=True), dict(packet, lease=True), dict(packet, seq=2**32)]:
            self.assertFalse(self.r.receive(bad, self.t)[0])

    def test_feedback_loss_latches_and_recovery_does_not_restart(self):
        self.arm()
        self.drive()
        self.advance(50_000, feedback=False)
        self.assertEqual(self.r.reason, "encoder_timeout")
        self.assertFalse(self.r.status()["feedback_valid"])
        self.advance(220_000)
        self.assertEqual(self.r.state, State.FAULT)
        self.assertEqual(self.r.target_speed, 0)

    def test_duplicate_feedback_does_not_keep_alive(self):
        self.arm()
        self.drive()
        for _ in range(5):
            self.t += 10_000
            self.r.update_encoder(self.t, sequence=self.sample, capture_us=self.t - 10_000,
                                  left_ticks=0, right_ticks=0)
            self.r.tick(self.t)
        self.assertEqual(self.r.reason, "encoder_timeout")

    def test_tick_loop_stall_is_a_fault(self):
        self.arm()
        self.drive()
        self.t += 60_000
        self.r.tick(self.t)
        self.assertEqual(self.r.reason, "control_deadline_missed")

    def test_communication_cannot_feed_control_watchdog(self):
        self.arm()
        self.drive()
        for _ in range(6):
            self.t += 10_000
            self.sample += 1
            self.r.update_encoder(self.t, sequence=self.sample, capture_us=self.t, left_ticks=0, right_ticks=0)
            self.r.offer(self.t)
            if self.r.armed:
                self.drive()
        self.assertEqual(self.r.reason, "control_deadline_missed")

    def test_offer_rejects_float_clock_equal_to_previous_integer(self):
        with self.assertRaises(ValueError):
            self.r.offer(float(self.t))

    def test_clock_regression(self):
        self.arm()
        self.drive()
        self.r.tick(self.t - 1)
        self.assertEqual(self.r.reason, "clock_regressed")
        self.assertEqual(self.r.target_speed, 0)

    def test_untrusted_clock_types(self):
        for bad in [True, -1, 1.5, float("inf")]:
            self.setUp()
            self.arm()
            self.r.tick(bad)
            self.assertFalse(self.r.armed)

    def test_lease_storage_bounded(self):
        for _ in range(1000):
            self.r.offer(self.t)
        self.assertLessEqual(len(self.r._leases), 32)

    def test_counter_capture_not_arrival_interval(self):
        self.r.update_encoder(self.t + 30_000, sequence=23, capture_us=self.t + 10_000,
                              left_ticks=100, right_ticks=200)
        scale = math.tau * .025 / 2048 / .01
        self.assertAlmostEqual(self.r.left_mps, 100 * scale)
        self.assertAlmostEqual(self.r.right_mps, 200 * scale)

    def test_counter_reset_not_negative_speed_command(self):
        self.arm()
        self.drive()
        self.t += 10_000
        self.r.update_encoder(self.t, sequence=23, capture_us=self.t,
                              left_ticks=-100000, right_ticks=0)
        self.assertEqual(self.r.reason, "encoder_implausible")

    def test_disarm_cancels_queued_drive(self):
        self.arm()
        packet, _ = self.drive()
        self.assertTrue(self.control("DISARM")[0])
        self.assertFalse(self.r.receive(packet, self.t)[0])
        self.assertEqual(self.r.target_speed, 0)

    def test_fuzz_never_drives_without_armed_state(self):
        for seed in range(20):
            self.setUp()
            rng = random.Random(seed)
            for _ in range(100):
                self.advance(10_000)
                event = rng.randrange(5)
                if event == 0:
                    self.r.set_estop(rng.choice([True, False]), self.t)
                elif event in (1, 2):
                    self.control("ARM" if event == 1 else "CLEAR")
                elif event == 3 and self.r.armed:
                    self.drive(rng.choice([0., 1.]))
                else:
                    self.r.receive({"v": 1, "kind": "STOP"}, self.t)
                if not self.r.armed:
                    self.assertEqual(self.r.target_speed, 0)


class WireAndPITests(unittest.TestCase):
    def test_fragmented_and_coalesced_frames(self):
        frames = encode({"v": 1, "kind": "STOP"}) + encode({"v": 1, "kind": "ARM"})
        d, found = Decoder(), []
        for byte in frames:
            found.extend(d.feed(bytes([byte])))
        self.assertEqual([x["kind"] for x in found], ["STOP", "ARM"])
        self.assertEqual(Decoder().feed(frames), found)

    def test_corrupt_frame_does_not_hide_following_stop(self):
        frame = bytearray(encode({"v": 1, "kind": "STOP"}))
        frame[-1] ^= 0xff
        d = Decoder()
        self.assertEqual(d.feed(b"boot log\n" + frame + encode({"v": 1, "kind": "STOP"})),
                         [{"v": 1, "kind": "STOP"}])
        self.assertGreater(d.errors, 0)

    def test_bad_length_and_reconnect(self):
        d = Decoder()
        d.feed(MAGIC + struct.pack("<H", MAX_PAYLOAD))
        d.reset()
        self.assertEqual(d.feed(MAGIC + b"\xff\xff" + encode({"v": 1})), [{"v": 1}])
        d.feed(b"x" * 65537)
        self.assertLessEqual(len(d.buffer), MAX_PAYLOAD + 8)

    def test_duplicate_keys_nan_and_nonobject_rejected(self):
        d = Decoder()
        for payload in [b'{"v":1,"v":2}', b'{"speed":NaN}', b'[]', b'{"speed":Infinity}']:
            body = struct.pack("<H", len(payload)) + payload
            self.assertEqual(d.feed(MAGIC + body + struct.pack("<I", zlib.crc32(body))), [])
        with self.assertRaises(ValueError):
            encode({"speed": float("nan")})

    def test_speed_loop_uses_average_and_has_no_two_motor_outputs(self):
        a, b = SpeedPI(), SpeedPI()
        self.assertEqual(a.update(1., .5, 1.5, .005, enabled=True),
                         b.update(1., 1., 1., .005, enabled=True))

    def test_integral_bounded_and_reset_on_stop(self):
        pi = SpeedPI()
        for _ in range(2000):
            value = pi.update(2.5, 0., 0., .005, enabled=True)
            self.assertLessEqual(abs(value.effort), 1)
        self.assertLessEqual(abs(pi.integral), 1)
        value = pi.update(0., 1., 1., .005, enabled=True)
        self.assertEqual(pi.integral, 0)
        self.assertEqual(value.effort, 0)
        self.assertTrue(value.brake_requested)

    def test_bad_dt_and_disabled_clear_effort(self):
        pi = SpeedPI()
        for dt in [0., -.1, .1, float("nan")]:
            value = pi.update(1., 0., 0., dt, enabled=True)
            self.assertEqual(value.effort, 0)
            self.assertTrue(value.brake_requested)
        self.assertEqual(pi.update(1., 0., 0., .005, enabled=False).effort, 0)

    def test_huge_integer_is_invalid_not_parser_crash(self):
        value = SpeedPI().update(10**400, 0., 0., .005, enabled=True)
        self.assertEqual(value.effort, 0)
        self.assertTrue(value.brake_requested)
        with self.assertRaises(ValueError):
            ContractConfig(max_speed_mps=10**400)

    def test_config_invalid(self):
        for args in [{"ticks_per_rev": True}, {"lease_us": 0}, {"max_speed_mps": float("nan")},
                     {"max_speed_mps": 10.}, {"settle_us": -1}]:
            with self.assertRaises(ValueError):
                ContractConfig(**args)


if __name__ == "__main__":
    unittest.main(verbosity=2)

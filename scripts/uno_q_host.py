#!/usr/bin/env python3
"""UNO Q MCU UART + USB 서보 호스트. 기본 진단, 주행은 /drive/safe와 명시 ARM만."""
import argparse
import json
import math
from pathlib import Path
import re
import signal
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))

from arena_vehicle_interface.clock_freshness import HostTimeWindow
from arena_vehicle_interface.serial_ingress import SerialIngress
from arena_vehicle_interface.steering_calibration import read_profile
from runtime_actuation_link import RuntimeActuationLink
from runtime_offer_adapter import adapt_runtime_payload
from same_host_command_clock import SameHostCommandClock, CommandClockError
from run_runtime_actuation_host import RuntimeHostOwner, bind_ros, jsonable, load_policy, load_motor_config, strict_json
from runtime_installed_profile import verify as verify_installed_profile
from uno_q_external_servo import UnoIngress, encode_status, serial_factory
from uno_q_servo import UsbServo, ServoProfile, NoServo, ServoMailbox
from uno_q_bench import OneShotBench
from uno_q_observations import SeparateAccelClock, EncoderObservationClock, steering_observation, bind_observations
from uno_q_profile import matching_mcu_profile, matching_motor_feedback, ProfileGatedMotorFeedback

DEFAULT_POLICY = ROOT/'config/hardware/uno_q_clock_policy.template.json'
ESV_PERIOD_NS = 20_000_000


class UnoHostOwner(RuntimeHostOwner):
    def __init__(self, *, servo, bench=None, steering_profile=None, expected_profile_hash=None, **options):
        super().__init__(**options)
        self.servo = servo
        self.bench = bench
        self.expected_profile_hash = expected_profile_hash
        self.profile_observation = None
        if self.motor_feedback is not None:
            self.motor_feedback = ProfileGatedMotorFeedback(self.motor_feedback,
                lambda status: self._matching_profile(self.mono(), status.get('boot')))
        self.steering_profile = steering_profile or (servo.profile.zero if getattr(servo, 'profile', None) else None)
        self.publish_accel = lambda stamp, meta: self.event('accel_observation', **meta)
        self.publish_steering = lambda row: self.event('steering_observation', **row)
        self.publish_encoder = lambda row: self.event('encoder_observation', **row)
        self.encoder_observation_clock = EncoderObservationClock()
        self.imu_clock = SeparateAccelClock(self.imu_clock, lambda stamp, meta: self.publish_accel(stamp, meta))
        self.last_steering_publish_ns = None
        self.arm_preparation = self.arm_ticket = self.authorization = self.candidate_drive = None
        self.status_sequence = 0
        self.last_status_ns = self.last_status_value = self.last_servo_fault = None

    def _cancel_arm(self, reason):
        if self.arm_preparation is not None:
            p, self.arm_preparation = self.arm_preparation, None
            result = dict(success=False, kind='ARM', reason=reason,
                          physical_output_verified=False, physical_stop_verified=False)
            self.event('servo_arm_preparation_completed', **result)
            if p['future'] is not None and not p['future'].done():
                p['future'].set_result(result)

    def _revoke_servo(self, reason):
        self._cancel_arm(reason)
        self.authorization = self.candidate_drive = None
        if self.servo.enabled or self.servo.authorization is not None:
            self.servo.release()
        self.last_status_value = None

    def fatal(self, reason):
        self._revoke_servo(reason)
        super().fatal(reason)

    def request_control(self, kind, *, future=None, shutdown=False):
        if kind != 'ARM':
            self._revoke_servo('explicit_'+kind.lower())
            return super().request_control(kind, future=future, shutdown=shutdown)
        session = self.engine.session
        offer = None if session is None else session.latest('offer')
        reason = None
        if not self.engine.drive_enabled or not self.servo.enabled:
            reason = 'drive_disabled_by_host'
        elif self.terminating or self.session_closed():
            reason = 'runner_terminating'
        elif self.arm_preparation is not None or self.authorization is not None:
            reason = 'ARM_already_pending_or_held'
        elif (session is None or offer is None or session.pending is not None or session.stop_requested
              or offer.payload['state'] != 'DISARMED'):
            reason = 'current_cleared_DISARMED_offer_required'
        elif not self._matching_profile(self.mono(), offer.payload['boot']):
            reason = 'fresh_compiled_MCU_profile_match_required'
        elif not self.servo.ready(self.mono()):
            reason = 'fresh_USB_servo_probe_required'
        if reason:
            result = dict(success=False, kind='ARM', reason=reason, physical_output_verified=False)
            self.event('servo_arm_rejected', **result)
            if future is not None and not future.done():
                future.set_result(result)
            return result
        auth = (offer.payload['boot'], offer.payload['generation'])
        self.arm_preparation = dict(authorization=auth, future=future,
                                    deadline_ns=self.mono()+self.policy.request_timeout_ns)
        try:
            self.servo.arm(auth, self.mono())
            self.authorization = auth
        except Exception as error:
            self._cancel_arm('servo_ARM_rejected:'+str(error));return None
        self.event('servo_arm_preparation_started', authorization=auth, automatic_center=False)
        return self.arm_preparation

    def _matching_profile(self, now, boot):
        observed = self.profile_observation
        return (self.expected_profile_hash is not None and observed is not None
                and observed['boot'] == boot and observed['mode'] == 'installed'
                and observed['hash'] == self.expected_profile_hash
                and observed['receipt_ns'] <= now < observed['receipt_ns']+100_000_000)

    def _accepted_goal(self, compact):
        candidate = self.candidate_drive
        if candidate is None or compact['state'] != 'ACTIVE':
            return
        auth = (compact['boot'], compact['generation'])
        if compact['accepted_seq'] != candidate['seq'] or auth != candidate['authorization'] or auth != self.authorization:
            return
        self.candidate_drive = None
        now = self.mono()
        if now >= candidate['deadline_ns']:
            self.fatal('accepted_servo_goal_expired');return
        try:
            self.servo.request_goal(seq=candidate['seq'], steering_rad=candidate['steering_rad'],
                authorization=auth, now=now, deadline_ns=candidate['deadline_ns'])
            self.event('servo_goal_after_MCU_acceptance', sequence=candidate['seq'],
                       steering_rad=candidate['steering_rad'], generation=auth[1])
        except Exception as error:
            self.fatal('servo_goal_rejected:'+str(error))

    def _uno_observation(self, row, receipt, processed_ns):
        required = {'type', 'v', 'boot', 'generation', 'accepted_seq', 'state', 'reason', 'bench_state', 'bench_seq', 'bench_reason', 'profile_sha256'}
        optional = {'mode', 'motor_counts', 'raw_count', 'pwm_duty_ppm', 'hal_fault', 'imu_ready',
                    'servo_ready', 'servo_applied_seq', 'uart_errors', 'imu_dropped', 'tx_imu_dropped', 'observation_seq', 'capture_us'}
        if (not required <= set(row) <= required|optional or type(row['v']) is not int or row['v'] != 1
                or not isinstance(row['boot'], str)
                or not isinstance(row['profile_sha256'], str) or re.fullmatch('[0-9a-f]{64}', row['profile_sha256']) is None
                or re.fullmatch('[0-9a-fA-F]{8}', row['boot']) is None
                or type(row['generation']) is not int or not 0 <= row['generation'] < 2**63
                or type(row['accepted_seq']) is not int or not -1 <= row['accepted_seq'] < 2**63
                or row['state'] not in ('DISARMED', 'ARMED', 'ACTIVE', 'ESTOP_LATCHED', 'FAULT_LATCHED')
                or type(row['bench_seq']) is not int or not -1 <= row['bench_seq'] < 2**63
                or row['bench_state'] not in ('disarmed', 'armed', 'running')
                or not isinstance(row['bench_reason'], str) or len(row['bench_reason']) > 64
                or not isinstance(row['reason'], str) or len(row['reason']) > 64):
            self.fatal('invalid_UNO_observation');return
        if processed_ns < receipt.monotonic_ns:
            self.fatal('UNO_observation_clock_order');return
        if processed_ns >= receipt.monotonic_ns+30_000_000:
            self.counts['stale_uno_observation'] += 1
            self.event('uno_observation_stale', receipt_monotonic_ns=receipt.monotonic_ns,
                       processed_monotonic_ns=processed_ns, freshness_extended=False)
            return
        for key in ('hal_fault', 'imu_ready', 'servo_ready'):
            if key in row and type(row[key]) is not bool:
                self.fatal('invalid_UNO_observation_'+key);return
        for key in ('motor_counts', 'raw_count', 'pwm_duty_ppm', 'servo_applied_seq', 'uart_errors', 'imu_dropped', 'tx_imu_dropped', 'observation_seq', 'capture_us'):
            if key in row and (type(row[key]) is not int or not -2**63 <= row[key] < 2**63):
                self.fatal('invalid_UNO_observation_'+key);return
        if 'mode' in row and row['mode'] not in ('bench', 'installed', 'unconfigured'):
            self.fatal('invalid_UNO_observation_mode');return
        self.event('uno_observation', payload=row, receipt_monotonic_ns=receipt.monotonic_ns,
                   session_authority=False)
        encoder = self.encoder_observation_clock.convert(row, receipt)
        if encoder is not None:self.publish_encoder(encoder)
        self.profile_observation = dict(boot=row['boot'], hash=row['profile_sha256'],
            mode=row.get('mode'), receipt_ns=receipt.monotonic_ns)
        if self.authorization is not None and not self._matching_profile(processed_ns, self.authorization[0]):
            self.fatal('compiled_MCU_profile_mismatch');return
        if self.bench is not None:
            self.bench.observe(row, receipt.monotonic_ns)
        session = self.engine.session
        offer = None if session is None else session.latest('offer')
        if (offer is None or row['boot'] != offer.payload['boot']
                or row['generation'] != offer.payload['generation']):
            return
        if row['state'] in ('FAULT_LATCHED', 'ESTOP_LATCHED'):
            self._revoke_servo('MCU_'+row['state']);return
        # 이 관측은 기존 ARM/lease/owner 증거를 대체하지 않고 DRIVE 수락 seq만 확인한다.
        if session.arm_barrier is not None and offer.payload['owner'] == self.engine.options['client_id']:
            self._accepted_goal(row)

    def _control_row(self, row, receipt, processed_ns):
        if row.get('type') == 'uno_q':
            self._uno_observation(row, receipt, processed_ns);return
        super()._control_row(row, receipt, processed_ns)
        if self.session_closed():
            return
        try:
            compact = adapt_runtime_payload(row, max_lease_us=self.policy.max_lease_us).compact
        except ValueError:
            return
        kind = compact['type']
        if kind not in ('offer', 'state', 'diag') or self.authorization is None:
            return
        auth, old = (compact['boot'], compact['generation']), self.authorization
        # ARM 한 번의 +1 전환만 현재 hold ACK를 유지한다. 다른 세대 전환은 폐기한다.
        if auth != old:
            if (self.arm_ticket is not None and self.arm_ticket.result is None
                    and auth == (old[0], old[1]+1) and compact['state'] in ('ARMED', 'ACTIVE')):
                try:
                    self.servo.rebind(old, auth);self.authorization = auth
                except Exception as error:
                    self.fatal('servo_generation_rebind:'+str(error));return
            else:
                self._revoke_servo('MCU_generation_changed');return
        if compact['state'] in ('FAULT_LATCHED', 'ESTOP_LATCHED'):
            self._revoke_servo('MCU_'+compact['state']);return
        if compact['state'] == 'DISARMED' and self.arm_preparation is None and self.arm_ticket is None:
            self._revoke_servo('MCU_disarmed');return
        if kind == 'offer' and compact['state'] in ('ARMED', 'ACTIVE'):
            if compact['owner'] != self.engine.options['client_id'] or not compact['output_ready']:
                self._revoke_servo('MCU_owner_or_readiness_changed');return
        if kind == 'diag':
            self._accepted_goal(compact)

    def drive(self, *, source_ros_ns, speed_mps, steering_rad):
        # 공용 owner와 같은 원천시각 보존 경로. USB 목표를 위해 accepted seq만 추가 추적한다.
        self.event('drive_received', source_ros_ns=source_ros_ns, speed_mps=speed_mps,
                   steering_rad=steering_rad, receipt_monotonic_ns=self.mono())
        if self.terminating or self.authorization is None or not self.servo.armed_ready(self.mono()):
            return
        if not self.capture_clock():
            return
        now = self.mono()
        if self.last_capture_after_ns <= now < self.last_capture_after_ns+self.policy.same_host.host_timestamp_error_ns:
            self.event('drive_source_rejected', reason='processing_now_within_capture_uncertainty');return
        try:
            source = self.clock.map_source(source_ros_ns, source_ros_epoch=self.clock.ros_epoch,
                                           now=HostTimeWindow(self.clock.host_epoch, now, now))
        except CommandClockError as error:
            self.event('drive_source_rejected', reason=error.reason, source_ros_ns=source_ros_ns)
            if error.clock_fault:
                self.fatal('same_host_clock:'+error.reason)
            return
        result = self.engine.drive(speed_mps=speed_mps, steering_rad=steering_rad,
                                    source=source.source, now_ns=now)
        self.event('drive_prepared', source=source, prepared=result)
        if result.get('wire') is not None:
            command = result['envelope']['command']
            deadline = min(now+60_000_000, source.source.lower_ns+self.policy.receiver_lease.source_ttl_ns)
            self.candidate_drive = dict(seq=command['seq'], steering_rad=steering_rad,
                authorization=(command['boot'], command['generation']), deadline_ns=deadline)
            try:
                self.io.send(result['wire']);self.counts['queued_DRIVE'] += 1
            except Exception as error:
                self.fatal('serial_send_failed:'+str(error))
        if self.session_closed():
            self.fatal('session_closed')

    def _external_status(self, now):
        if isinstance(self.servo, NoServo) or self.bench is not None:
            return
        session = self.engine.session
        offer = None if session is None else session.latest('offer')
        if offer is None or self.session_closed():
            return
        row = offer.payload
        if not offer.received.upper_ns <= now < offer.received.lower_ns+self.policy.receiver_lease.max_receipt_age_ns:
            return
        ready = (not self.terminating and self.servo.armed_ready(now)
                 and self.authorization == (row['boot'], row['generation']))
        applied = self.servo.applied_seq if ready else -1
        value = (row['boot'], row['generation'], row['lease'], ready, applied)
        if value == self.last_status_value and self.last_status_ns is not None and now-self.last_status_ns < ESV_PERIOD_NS:
            return
        observed = self.servo.observed_ns
        if observed is None or observed > now:
            age = 100_000
        else:
            ppm = self.policy.receiver_lease.max_rate_error_ppm
            age = ((now-observed)*1000+(1_000_000-ppm)-1)//(1_000_000-ppm)
        if age >= 100_000:
            ready = False;applied = -1
        self.status_sequence += 1
        self.io.send(encode_status(boot=row['boot'], generation=row['generation'], lease=row['lease'],
            status_seq=self.status_sequence, observation_age_us=min(age, 2**32-1), ready=ready,
            applied_drive_seq=applied))
        self.last_status_ns, self.last_status_value = now, value
        self.event('external_servo_status_queued', boot=row['boot'], generation=row['generation'],
                   lease=row['lease'], status_sequence=self.status_sequence, ready=ready,
                   applied_drive_seq=applied, observation_age_us=age)

    def tick(self):
        super().tick()
        now = self.mono();self.servo.tick(now)
        if self.last_steering_publish_ns is None or now-self.last_steering_publish_ns >= 20_000_000:
            self.publish_steering(steering_observation(self.servo, self.steering_profile, now))
            self.last_steering_publish_ns = now
        if self.servo.fault is not None and self.servo.fault != self.last_servo_fault:
            self.last_servo_fault = self.servo.fault;self.fatal('USB_servo_fault:'+self.servo.fault)
        if self.authorization is not None and self.servo.engaged and not self.servo.ready(now):
            self.fatal('USB_servo_observation_expired')
        if self.authorization is not None and not self._matching_profile(now, self.authorization[0]):
            self.fatal('compiled_MCU_profile_observation_expired')
        if self.candidate_drive is not None and now >= self.candidate_drive['deadline_ns']:
            self.fatal('MCU_drive_acceptance_not_observed_before_deadline')
        if self.arm_ticket is not None and self.arm_ticket.result is not None:
            ticket, self.arm_ticket = self.arm_ticket, None
            if not ticket.result['success']:
                self._revoke_servo('MCU_ARM_not_confirmed')
        if self.arm_preparation is not None:
            prep = self.arm_preparation
            if now >= prep['deadline_ns']:
                self._revoke_servo('servo_ARM_preparation_timeout')
            elif self.servo.armed_ready(now):
                offer = self.engine.session.latest('offer')
                if (offer is not None and offer.payload['output_ready'] and offer.payload['state'] == 'DISARMED'
                        and (offer.payload['boot'], offer.payload['generation']) == prep['authorization']):
                    self.arm_preparation = None
                    self.arm_ticket = super().request_control('ARM', future=prep['future'])
        try:
            self._external_status(now)
            if self.bench is not None:
                self.bench.step(self, now)
        except Exception as error:
            self.fatal('external_status_send_failed:'+str(error))

    def receive(self, *, timeout=0.):
        result = super().receive(timeout=timeout)
        # 공용 STOP drain에서도 USB ACK/torque OFF 순서기를 굶기지 않는다.
        if self.terminating:
            self.servo.tick(self.mono())
        return result

    def shutdown(self, reason):
        result = super().shutdown(reason)  # MCU STOP는 USB 장애와 무관하게 우선 송신/확인
        deadline = self.watchdog_ns()+100_000_000
        while self.watchdog_ns() < deadline and self.servo.fault is None and self.servo.release_pending:
            self.servo.tick(self.mono());time.sleep(.001)
        try:
            self.servo.io.close()
        finally:
            result['servo_torque_off_confirmed'] = self.servo.torque_off_confirmed
            result['servo_fault'] = self.servo.fault
            result['host_death_torque_off_guaranteed'] = False
        return result


def load_installed(path):
    manifest = verify_installed_profile(path)
    board = strict_json((Path(path)/'runtime_board.json').read_bytes())
    zero = read_profile(Path(path)/'steering_zero.json')
    if (board['motor_direction_confirmed'] is not True or board['encoder_direction'] not in (-1, 1)
            or any(type(board[k]) not in (float, int) or not math.isfinite(board[k]) or board[k] <= 0
                   for k in ('kp', 'ki', 'maximum_duty', 'maximum_speed_mps'))
            or board['maximum_duty'] > 1 or board['maximum_speed_mps'] > 13/3.6):
        raise ValueError('confirmed encoder/motor direction and finite installed gains/limits required')
    return board, ServoProfile(zero=zero, **board['servo']), manifest


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mcu-port')
    p.add_argument('--servo-port')
    p.add_argument('--output', type=Path)
    p.add_argument('--seconds', type=float, default=30.)
    p.add_argument('--clock-policy', type=Path, default=DEFAULT_POLICY)
    p.add_argument('--installed-profile-dir', type=Path)
    p.add_argument('--mcu-profile', type=Path, help='MCU 빌드에 사용한 uno_q_profile_v1 JSON; ARM fingerprint gate')
    p.add_argument('--steering-profile', type=Path,
                   default=ROOT/'config/hardware/steering_zero.assembled_2026_11_14.json',
                   help='진단 steering 관측의 실측 count 방향/영점; 운영 bundle을 지정하면 그 영점 우선')
    p.add_argument('--motor-feedback-config', type=Path)
    p.add_argument('--enable-drive-wire', action='store_true')
    p.add_argument('--diagnostics', action='store_true', help='기본값과 같은 출력 없는 진단 모드')
    p.add_argument('--bench', action='store_true', help='별도 bench firmware의 명시 단발 모터 시험; 서보 출력 없음')
    p.add_argument('--bench-duty-permille', type=int)
    p.add_argument('--bench-direction', type=int, choices=(0, 1), help='확인하려는 DIR 전기 레벨; 전진 부호를 가정하지 않음')
    p.add_argument('--bench-duration-ms', type=int)
    p.add_argument('--release-servo', action='store_true', help='진단에서도 torque OFF RAM 쓰기와 ACK 확인 허용')
    p.add_argument('--ros', action='store_true', help='ROS IMU/상태 발행 및 /drive/safe 서비스 연결')
    p.add_argument('--imu-topic', default='/hardware_runtime/imu')
    p.add_argument('--dry-run', action='store_true', help='장치/ROS/serial import 없이 설정·프레임 검사만')
    return p


def validate(args):
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 3600:
        raise ValueError('finite duration 1..3600 s required')
    policy, policy_record = load_policy(args.clock_policy)
    board = servo_profile = manifest = None
    if args.installed_profile_dir is not None:
        board, servo_profile, manifest = load_installed(args.installed_profile_dir)
    if args.enable_drive_wire and (board is None or not args.ros):
        raise ValueError('drive mode requires --ros and an explicit installed profile bundle')
    args.expected_profile_hash = None
    if args.enable_drive_wire or args.motor_feedback_config is not None:
        if args.mcu_profile is None:raise ValueError('runtime drive requires --mcu-profile fingerprint evidence')
        args.expected_profile_hash = matching_mcu_profile(args.mcu_profile, board)
    if args.enable_drive_wire and (args.diagnostics or args.bench):
        raise ValueError('runtime drive, diagnostics and raw bench are separate modes')
    if args.bench:
        if args.ros or args.diagnostics or args.release_servo:
            raise ValueError('bench is a separate motor-only mode without ROS command/servo output')
        if any(v is None for v in (args.bench_duty_permille, args.bench_direction, args.bench_duration_ms)):
            raise ValueError('bench requires explicit duty, DIR and duration')
        OneShotBench(duty_permille=args.bench_duty_permille, direction=args.bench_direction,
                     duration_us=args.bench_duration_ms*1000)
        if args.seconds < args.bench_duration_ms/1000+2:
            raise ValueError('runner duration must include pulse plus 2 s bounded setup/stop allowance')
    elif any(v is not None for v in (args.bench_duty_permille, args.bench_direction, args.bench_duration_ms)):
        raise ValueError('bench parameters require explicit --bench')
    if not args.dry_run:
        if not args.mcu_port or args.mcu_port == args.servo_port:
            raise ValueError('explicit MCU UART and distinct optional USB servo path required')
        if (args.enable_drive_wire or args.release_servo) and not args.servo_port:
            raise ValueError('runtime drive or torque release requires explicit USB servo path')
        if args.output is None or args.output.exists():
            raise ValueError('fresh --output directory required')
    if (re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*', args.imu_topic) is None
            or args.imu_topic in ('/drive', '/drive/safe', '/actuation/status', '/actuation/arm',
                                 '/actuation/clear_stop', '/actuation/disarm', '/actuation/stop')):
        raise ValueError('separate absolute observation topic required')
    return policy, policy_record, board, servo_profile, manifest


def main(argv=None):
    p = parser();args = p.parse_args(argv)
    try:
        policy, policy_record, board, servo_profile, manifest = validate(args)
        motor_feedback, motor_record = load_motor_config(args.motor_feedback_config)
        matching_motor_feedback(motor_record)
        selected_zero = servo_profile.zero if servo_profile else read_profile(args.steering_profile)
    except (ValueError, TypeError, KeyError, OSError, OverflowError) as error:
        p.error(str(error))
    if args.dry_run:
        example = encode_status(boot='1234abcd', generation=0, lease=1, status_seq=1,
                                observation_age_us=0, ready=False, applied_drive_seq=-1)
        print(json.dumps(dict(dry_run=True, hardware_opened=False, baudrate=1_000_000,
            drive_wire_enabled=args.enable_drive_wire, profile_present=board is not None,
            mode='bench' if args.bench else 'runtime' if args.enable_drive_wire else 'diagnostics',
            bench=None if not args.bench else dict(duty_permille=args.bench_duty_permille,
                direction=args.bench_direction, duration_ms=args.bench_duration_ms, automatic_repeat=False,
                servo_output=False),
            external_status_frame_hex=example.hex(), external_status_frame_bytes=len(example),
            policy=policy_record, physical_verified=False), ensure_ascii=False))
        return 0
    args.output.mkdir(parents=True, exist_ok=False)
    host_epoch = 'uno-host-'+uuid.uuid4().hex
    ros_epoch = 'uno-ros-'+uuid.uuid4().hex
    engine = RuntimeActuationLink(host_epoch=host_epoch, client_id='uno-'+uuid.uuid4().hex,
        timing=policy.receiver_lease, max_speed_mps=board['maximum_speed_mps'] if board else 1.,
        max_steering_rad=board['servo']['command_limit'] if board else .1,
        max_lease_us=policy.max_lease_us, request_timeout_ns=policy.request_timeout_ns,
        drive_enabled=args.enable_drive_wire)
    clock = SameHostCommandClock(host_epoch=host_epoch, ros_epoch=ros_epoch, bounds=policy.same_host)
    uart = UnoIngress(args.mcu_port)
    usb = SerialIngress(args.servo_port, factory=serial_factory) if args.servo_port else None
    if usb is not None:usb.commands = ServoMailbox()
    interrupted = {'reason': None}
    old_handlers = {}
    owner = node = executor = None
    ros_initialized = uart_started = False
    failure = None
    summary = {}
    (args.output/'registered.json').write_text(json.dumps(jsonable(dict(arguments=vars(args),
        clock_policy=policy_record, installed_profile=manifest, motor_feedback=motor_record,
        physical_verified=False, auto_arm=False)), indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    with (args.output/'events.jsonl').open('x', encoding='utf-8') as events, \
         (args.output/'uart_rx.bin').open('xb') as rx, (args.output/'uart_tx.bin').open('xb') as tx:
        def event(row):
            events.write(json.dumps(jsonable(row), allow_nan=False, ensure_ascii=False, separators=(',', ':'))+'\n')
            if row.get('event') in ('termination_requested', 'servo_fault', 'shutdown_completed'):
                events.flush();rx.flush();tx.flush()
        servo = UsbServo(usb, profile=servo_profile, enabled=args.enable_drive_wire, event=event) if usb else NoServo()
        bench = (OneShotBench(duty_permille=args.bench_duty_permille, direction=args.bench_direction,
                             duration_us=args.bench_duration_ms*1000) if args.bench else None)
        def stop_signal(number, _):
            interrupted['reason'] = 'signal_'+str(number)
        try:
            ros_now, mode = time.time_ns, lambda: ('SYSTEM_TIME', False)
            if args.ros:
                import rclpy
                from rclpy.node import Node
                from rclpy.executors import SingleThreadedExecutor
                from rclpy.callback_groups import ReentrantCallbackGroup
                from rclpy.signals import SignalHandlerOptions
                from rclpy.task import Future
                from rclpy.qos import qos_profile_sensor_data
                from sensor_msgs.msg import Imu, JointState
                from std_msgs.msg import String
                from std_srvs.srv import Trigger
                from ackermann_msgs.msg import AckermannDriveStamped
                rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO);ros_initialized = True
                node = Node('uno_q_actuation_host');ros_clock = node.get_clock()
                ros_now = lambda: ros_clock.now().nanoseconds
                mode = lambda: (ros_clock.clock_type.name, bool(getattr(ros_clock, 'ros_time_is_active', False)))
                if node.get_parameter('use_sim_time').value or mode() not in (('SYSTEM_TIME', False), ('ROS_TIME', False)):
                    raise RuntimeError('physical ROS clock required before opening hardware')
            owner = UnoHostOwner(servo=servo, bench=bench, steering_profile=selected_zero,
                expected_profile_hash=args.expected_profile_hash,
                engine=engine, ingress=uart, policy=policy, clock=clock,
                monotonic_ns=time.monotonic_ns, ros_now_ns=ros_now, read_mode=mode, event=event,
                raw_rx=rx, raw_tx=tx, motor_feedback=motor_feedback)
            if args.ros:
                executor = SingleThreadedExecutor();executor.add_node(node)
                bind_ros(node, owner, types=dict(Imu=Imu, JointState=JointState, String=String,
                    Trigger=Trigger, Drive=AckermannDriveStamped, sensor_qos=qos_profile_sensor_data),
                    callback_group=ReentrantCallbackGroup(), future_factory=Future, imu_topic=args.imu_topic)
                bind_observations(node, owner, Imu=Imu, String=String, sensor_qos=qos_profile_sensor_data)
            for number in (signal.SIGINT, signal.SIGTERM):
                old_handlers[number] = signal.signal(number, stop_signal)
            uart.start();uart_started = True
            owner.request_control('STOP')
            if usb is not None:usb.start()
            if args.release_servo:
                servo.release()
            began = time.monotonic()
            while not owner.terminating and interrupted['reason'] is None and time.monotonic()-began < args.seconds:
                if executor is not None:
                    executor.spin_once(timeout_sec=.005)
                else:
                    owner.tick();time.sleep(.005)
                if bench is not None and bench.done:break
            if bench is not None and not bench.done and not owner.terminating:
                owner.fatal('bench_session_ended_without_completion')
        except BaseException as error:
            failure = type(error).__name__+': '+str(error)
        finally:
            try:
                if owner is not None and uart_started:
                    summary['shutdown'] = owner.shutdown(failure or interrupted['reason'] or 'duration_complete')
                else:
                    uart.close();servo.io.close()
            except Exception as error:
                failure = failure or 'shutdown:'+str(error)
                uart.close();servo.io.close()
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if ros_initialized:
                rclpy.shutdown()
            for number, handler in old_handlers.items():
                signal.signal(number, handler)
            summary.update(error=failure, interruption=interrupted['reason'],
                exit_reason=None if owner is None else owner.exit_reason,
                servo_fault=servo.fault, servo_torque_off_confirmed=servo.torque_off_confirmed,
                servo_observation=servo.observation, counts={} if owner is None else dict(owner.counts),
                mcu_io=uart.stats, servo_io=None if usb is None else usb.stats,
                bench=None if bench is None else bench.result, drive_wire_enabled=args.enable_drive_wire,
                auto_arm=False, reconnected=False, source_timestamp_rewritten=False,
                physical_verified=False, host_death_torque_off_guaranteed=False)
            (args.output/'summary.json').write_text(json.dumps(jsonable(summary), indent=2,
                ensure_ascii=False)+'\n', encoding='utf-8')
    print(json.dumps(jsonable(summary), ensure_ascii=False))
    stopped = summary.get('shutdown') or {}
    stop_failed = uart_started and not (stopped.get('matching_stop_tx_receipt') and stopped.get('stop_ack_confirmed'))
    release_failed = (args.enable_drive_wire or args.release_servo) and not servo.torque_off_confirmed
    return int(failure is not None or servo.fault is not None or stop_failed or release_failed or
               (owner is not None and owner.exit_reason not in ('duration_complete',)))


if __name__ == '__main__':
    raise SystemExit(main())

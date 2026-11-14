"""명시 단발 모터 벤치. 별도 bench firmware에만 ARM/PULSE, 서보 출력은 전혀 없다."""
from concurrent.futures import Future
from uno_q_external_servo import encode_bench


class OneShotBench:
    def __init__(self, *, duty_permille, direction, duration_us):
        # validation only; bytes are sent after observed boot/current lease.
        encode_bench(boot='00000000', generation=0, lease=1, sequence=1, action=2,
                     direction=direction, duty_permille=duty_permille, duration_us=duration_us)
        self.duty, self.direction, self.duration = duty_permille, direction, duration_us
        self.phase = 'wait_startup_stop'
        self.sequence = 0
        self.pending_seq = None
        self.pulse_sequence = None
        self.bound = None
        self.started_ns = self.sent_ns = self.pulse_ns = self.last_keepalive_ns = None
        self.observation = self.observed_ns = None
        self.last_raw_count = self.stationary_since_ns = None
        self.clear_future = None
        self.done = False
        self.result = None

    def observe(self, row, receipt_ns):
        self.observation, self.observed_ns = dict(row), receipt_ns
        count = row.get('raw_count')
        if type(count) is not int:
            self.last_raw_count = self.stationary_since_ns = None
        elif self.last_raw_count != count:
            self.last_raw_count, self.stationary_since_ns = count, receipt_ns

    def _fail(self, owner, reason):
        self.done = True
        self.result = dict(success=False, reason=reason, physical_stop_verified=False)
        owner.event('bench_completed', **self.result)
        owner.fatal('bench:'+reason)

    def _send(self, owner, offer, action, now):
        self.sequence += 1
        wire = encode_bench(boot=offer['boot'], generation=offer['generation'], lease=offer['lease'],
            sequence=self.sequence, action=action, direction=self.direction,
            duty_permille=self.duty if action == 2 else 0, duration_us=self.duration if action == 2 else 0)
        owner.io.send(wire)
        self.sent_ns, self.pending_seq = now, self.sequence
        owner.event('bench_command_queued', action=action, sequence=self.sequence,
            generation=offer['generation'], duty_permille=self.duty if action == 2 else 0,
            duration_us=self.duration if action == 2 else 0, steering_output=False)

    def step(self, owner, now):
        if self.done or owner.terminating:return
        if self.started_ns is None:self.started_ns = now
        if self.phase != 'running' and now-self.started_ns >= 1_000_000_000:
            self._fail(owner, 'bench_setup_timeout');return
        session = owner.engine.session
        receipt = None if session is None else session.latest('offer')
        obs = self.observation
        if receipt is None or obs is None:
            if now-self.started_ns >= 1_000_000_000:self._fail(owner, 'initial_observation_timeout')
            return
        offer = receipt.payload
        if obs.get('mode') != 'bench':
            self._fail(owner, 'explicit_bench_firmware_required');return
        if now-self.observed_ns >= 100_000_000:
            self._fail(owner, 'MCU_bench_observation_expired');return
        if not receipt.received.upper_ns <= now < receipt.received.lower_ns+80_000_000:
            self._fail(owner, 'MCU_offer_expired');return
        if (obs['boot'], obs['generation']) != (offer['boot'], offer['generation']):return
        if obs.get('hal_fault') is True:
            self._fail(owner, 'MCU_HAL_fault');return
        if self.phase == 'wait_startup_stop':
            if not any(r['kind'] == 'STOP' and r['success'] for r in owner.completed):return
            if offer['state'] in ('ESTOP_LATCHED', 'FAULT_LATCHED'):
                self.clear_future = Future()
                owner.request_control('CLEAR', future=self.clear_future)
                self.phase = 'clear';return
            if offer['state'] != 'DISARMED':
                self._fail(owner, 'not_disarmed');return
            self.phase = 'arm'
        if self.phase == 'clear':
            if not self.clear_future.done():return
            if not self.clear_future.result()['success']:
                self._fail(owner, 'explicit_clear_unconfirmed');return
            self.phase = 'arm'
        if self.phase == 'arm':
            if offer['state'] != 'DISARMED':return
            if self.stationary_since_ns is None or now-self.stationary_since_ns < 200_000_000:return
            if obs['bench_state'] != 'disarmed':
                self._fail(owner, 'previous_bench_still_active');return
            self.bound = (offer['boot'], offer['generation'])
            self.sequence = max(self.sequence, obs['bench_seq'])
            self._send(owner, offer, 1, now);self.phase = 'wait_arm';return
        if self.bound != (offer['boot'], offer['generation']) or offer['state'] != 'DISARMED':
            self._fail(owner, 'bench_generation_or_state_changed');return
        if self.phase == 'wait_arm':
            if obs['bench_state'] == 'armed' and obs['bench_seq'] == self.pending_seq:
                self._send(owner, offer, 2, now)
                self.pulse_sequence = self.sequence
                self.pulse_ns = self.last_keepalive_ns = now;self.phase = 'running';return
            if now-self.sent_ns >= 100_000_000:self._fail(owner, 'bench_ARM_unconfirmed')
            return
        if self.phase == 'running':
            if obs['bench_state'] == 'disarmed':
                if obs['bench_reason'] == 'duration_complete' and obs['bench_seq'] >= self.pulse_sequence:
                    self.done = True
                    self.result = dict(success=True, reason='duration_complete_observed',
                        duty_permille=self.duty, duration_us=self.duration, direction=self.direction,
                        physical_stop_verified=False, automatic_repeat=False, steering_output=False)
                    owner.event('bench_completed', **self.result)
                elif obs['bench_seq'] >= self.pulse_sequence:
                    self._fail(owner, 'bench_ended:'+obs['bench_reason'])
                return
            if obs['bench_seq'] < self.pulse_sequence and now-self.pulse_ns >= 100_000_000:
                self._fail(owner, 'bench_PULSE_unconfirmed');return
            if now >= self.pulse_ns+self.duration*1000+150_000_000:
                self._fail(owner, 'duration_completion_unconfirmed');return
            if now-self.last_keepalive_ns >= 20_000_000:
                self._send(owner, offer, 4, now);self.last_keepalive_ns = now

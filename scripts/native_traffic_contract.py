"""교통 평가용 C1 원천 배열 대조. 운영 시각 승인이나 제어 입력을 만들지 않는다."""
from bisect import bisect_left
import math

from arena_vehicle_interface.native_lidar_sampling import NativeRevolutionAssembler


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class NativeSamplingAudit:
    def __init__(self, profile, rate_hz):
        if (profile.get('schema') != 'c1_angular_templates_v1'
                or profile.get('time_model') != 'period_divided_by_count'
                or profile.get('hardware_timing_verified') is not False
                or not finite(rate_hz) or profile.get('rotation_rate_hz') != rate_hz):
            raise ValueError('native sampling profile/time/rate mismatch')
        model = NativeRevolutionAssembler(profile['patterns_q14'], rate_hz=rate_hz)
        self.patterns = model.patterns
        self.period = model.period
        self.angle_error = math.radians(.0361)  # 5000 source rays, unchanged quantization budget

    def source_payload_ok(self, row):
        """완성 배열 자체의 형식·각도/시간. 과거 취득 누락 누계와 구분한다."""
        index = row.get('template_index')
        if type(index) is not int or not 0 <= index < len(self.patterns):
            return False
        n = len(self.patterns[index])
        first, last = row.get('first_ray_stamp_s'), row.get('last_ray_stamp_s')
        capture, angular = row.get('max_capture_time_error_s'), row.get('max_source_angle_quantization_rad')
        return (row.get('model') == 'native_sequential_v1'
                and row.get('time_model') == 'period_divided_by_count'
                and row.get('hardware_timing_verified') is False
                and type(row.get('samples')) is int and row['samples'] == n
                and all(finite(v) for v in (first, last, capture, angular))
                and first >= 0 and abs(last-first-self.period*(n-1)/n) <= 1e-8
                and 0 <= capture <= .0021001 and 0 <= angular <= self.angle_error)

    def source_ok(self, row):
        # 기존 전체 판정 유지: 완성 배열이 맞더라도 취득 유실은 실패다.
        return self.source_payload_ok(row) and row.get('discarded_source_gaps') == 0

    def received_ok(self, row, source):
        if not self.source_ok(source):
            return False
        return self.received_payload_ok(row, source)

    def received_payload_ok(self, row, source):
        if not self.source_payload_ok(source):
            return False
        pattern = self.patterns[source['template_index']]
        n = len(pattern)
        stamps, angles, ranges = (row.get(k) for k in ('acquired_at_s', 'angles_rad', 'ranges'))
        if (any(not isinstance(v, list) or len(v) != n for v in (stamps, angles, ranges))
                or row.get('representation') != 'native_measurements_v1'
                or row.get('time_basis') != 'simulation_acquisition'
                or row.get('source_frame_id') != 'laser_frame'
                or type(row.get('source_epoch')) is not int or row['source_epoch'] < 0
                or row.get('time_increment') is not None
                or not finite(row.get('stamp_s'))
                or abs(row['stamp_s']-source['first_ray_stamp_s']) > 1e-8):
            return False
        for i, (when, angle, distance) in enumerate(zip(stamps, angles, ranges)):
            if (not finite(when) or not finite(angle)
                    or abs(when-(source['first_ray_stamp_s']+self.period*i/n)) > 1e-8
                    or abs(math.remainder(angle-pattern[i], math.tau)) > self.angle_error
                    or not (distance is None or distance in ('+inf', '-inf') or finite(distance))):
                return False
        return True

    def audit(self, records, received):
        bad_sources = [i for i, r in enumerate(records) if not self.source_ok(r)]
        valid = [r for r in records if self.source_ok(r)]
        times = [r['first_ray_stamp_s'] for r in valid]
        monotonic = all(b > a for a, b in zip(times, times[1:]))
        missing, bad_received = [], []
        previous = -math.inf
        for i, row in enumerate(received):
            t = row.get('stamp_s')
            if not finite(t) or t <= previous or not monotonic:
                bad_received.append(i)
                continue
            previous = t
            j = bisect_left(times, t-1e-8)
            if j == len(times) or abs(times[j]-t) > 1e-8:
                missing.append(i)
            elif not self.received_ok(row, valid[j]):
                bad_received.append(i)
        return dict(verified=bool(records) and bool(received) and not bad_sources
                    and monotonic and not missing and not bad_received,
                    source_errors=bad_sources[:20], invalid_received=bad_received[:20],
                    unmatched_received=missing[:20], acquisition_records=len(records),
                    received_scans=len(received), sample_counts=sorted({r['samples'] for r in valid}),
                    time_model='period_divided_by_count', hardware_timing_verified=False,
                    completed_payload=self._completed_payload_audit(records, received))

    def _completed_payload_audit(self, records, received):
        """기존 실패를 유지하며 완성→수신 대응/취득 누락을 따로 보고한다.

        수신 기록창이 source 기록창 전체를 포함하는지 모르므로 미수신 수를
        DDS 유실로 단정하지 않는다. 누계값을 0으로 덮은 진단 사본도 만들지 않는다.
        """
        valid = [r for r in records if self.source_payload_ok(r)]
        times = [r['first_ray_stamp_s'] for r in valid]
        monotonic = all(b > a for a,b in zip(times,times[1:]))
        matched, invalid, unmatched = set(), [], []
        previous = -math.inf
        for i,row in enumerate(received):
            t = row.get('stamp_s')
            if not finite(t) or t <= previous or not monotonic:
                invalid.append(i)
                continue
            previous = t
            j = bisect_left(times,t-1e-8)
            if j == len(times) or abs(times[j]-t)>1e-8:
                unmatched.append(i)
            elif not self.received_payload_ok(row,valid[j]) or j in matched:
                invalid.append(i)
            else:
                matched.add(j)
        counters = [r.get('discarded_source_gaps') for r in records]
        counter_valid = (bool(counters) and all(type(v) is int and v>=0 for v in counters)
                         and all(b>=a for a,b in zip(counters,counters[1:])))
        corresponding = bool(records) and bool(received) and len(valid)==len(records) and monotonic and not invalid and not unmatched
        return dict(received_arrays_match_completed_sources=bool(corresponding),
            all_completed_sources_received=bool(corresponding and len(matched)==len(valid)),
            valid_completed_sources=len(valid), matched_completed_sources=len(matched),
            unreceived_completed_sources=len(valid)-len(matched),
            invalid_received=invalid[:20], unmatched_received=unmatched[:20],
            acquisition_counter_valid=bool(counter_valid),
            acquisition_loss_free=bool(counter_valid and all(v==0 for v in counters)),
            acquisition_gap_count=counters[-1] if counter_valid else None,
            scope='completed payload correspondence only; not loss-free acquisition or downstream callback proof')


def native_received_row(scan):
    return dict(stamp_s=scan.acquired_at_s[0], time_increment=None,
        representation='native_measurements_v1', time_basis=scan.time_basis,
        source_frame_id=scan.source_frame_id, source_epoch=scan.source_epoch,
        acquired_at_s=list(scan.acquired_at_s), angles_rad=list(scan.angles_rad),
        ranges=[float(v) if math.isfinite(v) else ('+inf' if v == math.inf else '-inf' if v == -math.inf else None)
                for v in scan.ranges])

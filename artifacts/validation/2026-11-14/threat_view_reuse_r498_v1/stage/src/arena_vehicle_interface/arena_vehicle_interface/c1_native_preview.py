"""C1 SDK native 순서의 진단 변환. 추정 시각을 실물 취득 시각으로 승인하지 않는다."""
from collections import Counter
import math


def integer(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError('invalid ' + name)
    return value


class C1NativePreview:
    time_basis = 'unverified_estimate'
    estimator = 'sdk_host_first_plus_nominal_200us'

    def __init__(self, *, source_epoch, frame_id='c1_unmounted_preview'):
        self.epoch = integer(source_epoch, 0, 2**63-1, 'epoch')
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise ValueError('empty frame')
        self.frame_id = frame_id
        self.last = None
        self.counts = Counter()

    def convert(self, batch, *, receipt_monotonic_ns, receipt_ros_ns):
        """수신 시계 차이만 맞춘다. SDK 지연/점 주기/배치 겹침을 교정하지 않는다."""
        if set(batch) != {'schema', 'scan', 'begin_monotonic_ns', 'end_monotonic_ns',
                          'sdk_estimated_first_us', 'us_per_sample', 'nodes'}:
            raise ValueError('invalid SDK batch fields')
        if batch['schema'] != 'c1_sdk_native_preview_v1' or batch['us_per_sample'] != 200.:
            raise ValueError('only measured Standard nominal 200us supported')
        maximum = 2**63-1
        scan = integer(batch['scan'], 0, maximum, 'scan')
        begin = integer(batch['begin_monotonic_ns'], 0, maximum, 'begin')
        end = integer(batch['end_monotonic_ns'], begin, maximum, 'end')
        first = integer(batch['sdk_estimated_first_us'], 1, maximum//1000, 'SDK first')*1000
        receipt = integer(receipt_monotonic_ns, end, maximum, 'receipt')
        ros = integer(receipt_ros_ns, 1, maximum, 'ROS clock')
        if first > end:
            raise ValueError('SDK first exceeds host completion')
        nodes = batch['nodes']
        if not isinstance(nodes, list) or not 2 <= len(nodes) <= 8192:
            raise ValueError('invalid node count')
        angles, ranges = [], []
        for node in nodes:
            if not isinstance(node, list) or len(node) != 4:
                raise ValueError('invalid HQ node')
            angle, distance, quality, flag = [integer(v, 0, top, name) for v, top, name in
                zip(node, (65535, 2**32-1, 255, 255), ('angle', 'distance', 'quality', 'flag'))]
            angles.append(math.pi-angle*math.pi/32768.)
            # SDK 거리0은 제조사 ROS와 같은 무반환(+Inf). 자유 공간은 아니다.
            # 양수 거리/품질0은 무효(NaN)로 구분한다. 원 정수는 raw에 보존한다.
            ranges.append('+inf' if distance == 0 else distance/4000. if quality else None)
        offset = ros-receipt
        last = first+(len(nodes)-1)*200_000
        reset = None
        overlap = 0
        if self.last is not None:
            old_scan, old_first, old_last, old_receipt, old_offset = self.last
            if receipt <= old_receipt or first <= old_first or scan <= old_scan:
                reset = 'source_clock_or_sequence_restart'
            elif abs(offset-old_offset) > 100_000_000:
                reset = 'ros_clock_step'
            else:
                overlap = max(0, old_last-first)
        if first+offset < 0:
            raise ValueError('negative mapped first stamp')
        if reset:
            self.epoch += 1
            self.counts[reset] += 1
        stamps = [(first+offset+i*200_000)/1e9 for i in range(len(nodes))]
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise ValueError('mapped stamp precision insufficient')
        self.last = (scan, first, last, receipt, offset)
        self.counts['batches'] += 1
        self.counts['points'] += len(nodes)
        self.counts['nominal_overlap_batches'] += bool(overlap)
        self.counts['nominal_after_receipt_batches'] += last > receipt
        payload = dict(schema='arena_native_lidar_v1', ranges_m=ranges, angles_rad=angles,
            acquired_at_s=stamps, source_frame_id=self.frame_id, source_epoch=self.epoch,
            range_min_m=.05, range_max_m=12., time_basis=self.time_basis)
        diagnostic = dict(time_contract_verified=False, time_estimator=self.estimator,
            sensor_acquisition_timestamp=False, source_epoch=self.epoch, reset_reason=reset,
            nominal_overlap_ns=overlap, nominal_end_after_receipt_ns=max(0, last-receipt),
            sdk_first_age_at_receipt_ns=receipt-first, transport_after_grab_ns=receipt-end,
            clock_mapping='same_host_monotonic_to_ros_receipt_offset', points=len(nodes),
            mounting_calibrated=False, drive_output=False,
            range_encoding='sdk_zero_distance_nonreturn_v1',
            zero_distance_points=sum(value == '+inf' for value in ranges),
            invalid_quality_positive_distance_points=sum(value is None for value in ranges),
            nonreturn_implies_free_space=False)
        return payload, diagnostic

#!/usr/bin/env python3
"""저장 센서의 입력 형식/유효성 표시를 읽는다. 제어·교정·시각 보정 없음."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

from audit_sensor_probe import audit
from arena_vehicle_interface.drive_feedback import MOTOR_TOPIC, parse_motor_state

SUPPORTED = {'/imu/data', '/scan', '/camera/color/image_raw', '/camera/color/camera_info', MOTOR_TOPIC}


class InputInspection:
    """값을 바꾸지 않는 상수 크기 요약. frame/layout 목록은 각 16개로 제한."""
    def __init__(self, expected_frames=None):
        self.expected_frames = dict(expected_frames or {})
        for topic, frame in self.expected_frames.items():
            if topic not in SUPPORTED or type(frame) is not str or not frame.strip() or len(frame) > 128:
                raise ValueError('known topic and explicit short frame name required')
        self.topics = {}

    def observe(self, row, msg):
        topic, index = row['topic'], row['index']
        if topic not in SUPPORTED:
            raise ValueError('input inspection requires current single-motor sensor topics')
        r = self.topics.setdefault(topic, dict(count=0, findings=Counter(), examples={},
            frames=Counter(), frame_overflow_samples=0, layouts=Counter(), layout_overflow_samples=0,
            values={}, scan_returns=Counter()))
        r['count'] += 1
        def finding(name):
            r['findings'][name] += 1
            examples = r['examples'].setdefault(name, [])
            if len(examples) < 3:
                examples.append(index)
        def category(field, value, overflow):
            # Truncation is not allowed to merge distinct names into one frame/layout.
            if len(value) <= 128 and (value in r[field] or len(r[field]) < 16):
                r[field][value] += 1
            else:
                r[overflow] += 1
        def finite_values(name, values):
            for value in values:
                value = float(value)
                if not math.isfinite(value):
                    finding(name+'_nonfinite')
                    return False
            return True
        def extent(name, value):
            if math.isfinite(value):
                item = r['values'].setdefault(name, dict(minimum=value, maximum=value))
                item['minimum'] = min(item['minimum'], value)
                item['maximum'] = max(item['maximum'], value)

        frame = msg.header.frame_id
        category('frames', frame, 'frame_overflow_samples')
        if not frame.strip():
            finding('empty_frame')
        if topic in self.expected_frames and frame != self.expected_frames[topic]:
            finding('declared_frame_mismatch')
        if not 0 <= msg.header.stamp.nanosec < 1_000_000_000:
            finding('header_nanosec_out_of_range')
        if row['source_ns'] <= 0:
            finding('nonpositive_header_time')

        if topic == MOTOR_TOPIC:
            try:
                position, velocity = parse_motor_state(msg.name, msg.position, msg.velocity)
            except ValueError:
                finding('single_motor_format_or_nonfinite')
            else:
                extent('motor_position_rad', position)
                extent('motor_velocity_rad_s', velocity)
            # No wheel/body velocity conversion or encoder-count reconstruction.
        elif topic == '/imu/data':
            for label, vector, covariance in (
                ('gyro', msg.angular_velocity, msg.angular_velocity_covariance),
                ('accel', msg.linear_acceleration, msg.linear_acceleration_covariance),
            ):
                if covariance[0] == -1:
                    finding(label+'_unavailable')
                elif all(v == 0 for v in covariance):
                    finding(label+'_covariance_unknown')
                finite_values(label+'_covariance', covariance)
                values = (vector.x, vector.y, vector.z)
                finite_values(label, values)
                for axis, value in zip(('x', 'y', 'z'), values):
                    extent(label+'_'+axis, float(value))
        elif topic == '/camera/color/image_raw':
            category('layouts', f'{msg.width}x{msg.height}:{msg.encoding}:step={msg.step}', 'layout_overflow_samples')
            # Same layout boundary as VisionPipeline.pump; no inference/worker is started.
            if (msg.encoding not in ('rgb8', 'bgr8') or msg.height <= 0 or msg.width <= 0
                    or msg.step < msg.width*3 or len(msg.data) != msg.height*msg.step):
                finding('vision_layout_unsupported')
            else:
                extent('row_padding_bytes', msg.step-msg.width*3)
        elif topic == '/camera/color/camera_info':
            category('layouts', f'{msg.width}x{msg.height}:{msg.distortion_model}', 'layout_overflow_samples')
            if msg.width <= 0 or msg.height <= 0:
                finding('camera_info_empty_dimensions')
            if len(msg.k) == 9 and msg.k[0] == 0:
                finding('camera_info_uncalibrated')
            if len(msg.k) != 9 or len(msg.r) != 9 or len(msg.p) != 12:
                finding('camera_info_matrix_shape')
            finite_values('camera_info', (*msg.k, *msg.r, *msg.p, *msg.d))
        elif topic == '/scan':
            finite = finite_values('scan_metadata', (msg.angle_min, msg.angle_max, msg.angle_increment,
                msg.range_min, msg.range_max, msg.time_increment, msg.scan_time))
            extent('ranges_per_message', len(msg.ranges))
            if (not finite or len(msg.ranges) < 2 or msg.angle_increment <= 0
                    or not 0 <= msg.range_min < msg.range_max
                    or msg.time_increment < 0 or msg.scan_time < 0):
                finding('scan_layout_invalid')
            elif abs((msg.angle_min+(len(msg.ranges)-1)*msg.angle_increment)-msg.angle_max) > max(1e-5, abs(msg.angle_increment)*1e-3):
                # Float32 metadata rounding allowance, NOT a driving/visibility threshold.
                finding('scan_endpoint_inconsistent')
            if len(msg.intensities) not in (0, len(msg.ranges)):
                finding('scan_intensity_length_mismatch')
            valid_bounds = math.isfinite(msg.range_min) and math.isfinite(msg.range_max) and 0 <= msg.range_min < msg.range_max
            for value in msg.ranges:
                key = ('nan' if math.isnan(value) else 'positive_inf' if value == math.inf else
                       'negative_inf' if value == -math.inf else 'finite_bounds_invalid' if not valid_bounds else
                       'below_minimum' if value < msg.range_min else
                       'above_maximum' if value > msg.range_max else 'finite_in_range')
                r['scan_returns'][key] += 1
            if msg.time_increment == 0:
                finding('zero_time_increment')
            extent('time_increment_s', float(msg.time_increment))
            extent('scan_time_s', float(msg.scan_time))

    def result(self):
        return dict(per_topic=self.topics,
                    declared_expected_frames=self.expected_frames,
                    missing_declared_topics=sorted(set(self.expected_frames)-set(self.topics)),
                    unchecked_frame_topics=sorted(set(self.topics)-set(self.expected_frames)),
                    interpretation=dict(
                        input_values_modified=False, controller_replayed=False, controller_compatibility_verified=False,
                        sensor_acquisition_time_verified=False, sensor_units_or_axes_verified=False,
                        calibration_applied=False, hardware_verified=False, ready_to_drive=False,
                        notes=[
                            'Findings are observations, not a new controller acceptance threshold.',
                            'Frame string equality does not prove mounting or axes. Empty declaration skips comparison.',
                            'Unknown covariance, uncalibrated CameraInfo, no-return and zero timing may be valid producer states; do not fill or correct them.',
                            'LiDAR no-return is not free space. This summary does not certify front/side coverage.',
                            'Positive timestamps and nominal values do not prove actual acquisition timing, units, motor counts or slip.',
                            'Image metadata cannot detect mislabeled BGR/RGB pixels; no image/CameraInfo pairing or optical calibration is certified.',
                        ]))


def analyze(recording, expected_frames=None):
    inspection = InputInspection(expected_frames)
    integrity = audit(recording, on_message=inspection.observe)
    return dict(schema='arena_sensor_probe_inputs_v1', record_integrity=integrity, **inspection.result())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--expect-frame', action='append', default=[], metavar='TOPIC=FRAME')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    frames = {}
    for value in args.expect_frame:
        topic, sep, frame = value.partition('=')
        if not sep or topic in frames:
            raise ValueError('unique TOPIC=FRAME required')
        frames[topic] = frame
    result = analyze(args.recording, frames)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(schema=result['schema'], topics=len(result['per_topic']), ready_to_drive=False)))


if __name__ == '__main__':
    main()

"""C1 정렬/발행과 운동 보정을 함께 대조하는 PC 전용 분석기.

고정된 공개 C++ 원문 함수만 실행하며 SDK 통신/ROS/장치는 실행하지 않는다.
정답 pose/점 식별자는 평가기에만 존재한다. 출력/관문/운영 설정 변경 없음.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess

import numpy as np

from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.lidar_point_review import project_acquisition_points

ROOT = Path(__file__).resolve().parents[1]
COMMIT = '34300099fadfc772965962dec837bf436706188f'
PINNED = {
    'src/sllidar_node.cpp': 'f414bd77520de7d1ba06bd21b7f4515f065d4f775ce6714bbb8bcb046c0100ad',
    'sdk/src/sl_lidar_driver.cpp': '668806b85a365e823db4247992ac559f20ea66dbba30fdd3ec4f8071ee672b2a',
    'LICENSE': '3475642aaf47837729092f181f151866abb6e63b624cf504a9229d7641d2eaf7',
}
CONFIG = MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675,
                      lidar_x_m=.06, lidar_y_m=0., lidar_yaw_rad=0.)
COUNT, HZ, START, NOW = 500, 10., 1., 1.15
ROOM = np.array([3., 1.5])  # x=+-3, y=+-1.5 m, analytic fixture, NOT official track


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def block(source, marker):
    """고정 원문 전용 추출기. 범용 문법 분석기가 아니며 표식은 유일해야 한다."""
    if source.count(marker) != 1:
        raise ValueError('source marker not unique: '+marker)
    start = source.index(marker)
    brace = source.index('{', start)
    depth, end = 1, brace+1
    while depth:
        if source[end] == '{':
            depth += 1
        elif source[end] == '}':
            depth -= 1
        end += 1
    return source[start:end], source[brace+1:end-1]


def build_native(upstream, out):
    contents = {}
    for name, expected in PINNED.items():
        path = upstream/name
        if digest(path) != expected:
            raise ValueError('pinned upstream hash mismatch: '+name)
        contents[name] = path.read_text()
    sdk, driver = contents['sdk/src/sl_lidar_driver.cpp'], contents['src/sllidar_node.cpp']
    selected = [block(sdk, marker)[0] for marker in (
        'static inline float getAngle(const sl_lidar_response_measurement_node_hq_t& node)',
        'static inline void setAngle(sl_lidar_response_measurement_node_hq_t& node, float v)',
        'static inline sl_u32 getDistanceQ2(const sl_lidar_response_measurement_node_hq_t& node)',
        'template <class TNode>\n    static bool angleLessThan',
        'template < class TNode >\n    static sl_result ascendScanData_')]
    selected.append(block(driver, 'void publish_scan(')[0])
    _, body = block(driver, 'if (angle_compensate)')
    wrapper = '''
void compensate(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& scan_pub,
  sl_lidar_response_measurement_node_hq_t* nodes, size_t count,
  size_t angle_compensate_multiple, double start, double scan_duration) {
  const rclcpp::Time start_scan_time{start};
  const bool inverted=false;
  const float angle_min=0, angle_max=2*M_PI, max_distance=12;
  const std::string frame_id="laser";
''' + body + '\n}\n'
    (out/'vendor_functions.inc').write_text('\n'.join(selected)+wrapper)
    (out/'VENDOR_LICENSE.txt').write_text(contents['LICENSE'])
    (out/'THIRD_PARTY_NOTICES.md').write_text(
        '# 원문 함수\n\nSlamtec sllidar_ros2 '+COMMIT+
        '. vendor_functions.inc의 선택 함수/분기 본문은 원문 그대로입니다.\n'
        '출처: https://github.com/Slamtec/sllidar_ros2/tree/'+COMMIT+
        '\nBSD-2-Clause: [VENDOR_LICENSE.txt](VENDOR_LICENSE.txt). 타입/전송 stub은 팀 시험 코드입니다.\n')
    shutil.copyfile(ROOT/'tests/fixtures/c1_scan_pipeline_stub.cpp', out/'stub.cpp')
    command = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-pedantic',
               '-fsanitize=undefined', '-fno-sanitize-recover=all',
               str(out/'stub.cpp'), '-o', str(out/'probe')]
    result = subprocess.run(command, capture_output=True, text=True, timeout=60)
    (out/'compile.stdout').write_text(result.stdout)
    (out/'compile.stderr').write_text(result.stderr)
    (out/'compile_exit.txt').write_text(str(result.returncode)+'\n')
    if result.returncode:
        raise RuntimeError('native compile failed; logs preserved')
    return out/'probe'


def pose(t, v, w):
    """기준 rear axle의 독립 폐형식. MotionHistory/advance_local을 호출하지 않는다."""
    angle = w*t
    c, s = math.cos(angle), math.sin(angle)
    p = np.array([v*t, 0.]) if w == 0 else np.array([v/w*s, v/w*(1-c)])
    rot = np.array([[c, -s], [s, c]])
    return p+rot@np.array([-CONFIG.rear_axle_x_m, 0.]), rot


def scene(v, w, phase_deg, missing=False):
    times = START+np.arange(COUNT)/(COUNT*HZ)
    # Start pose at t=0 relative to this scan, keeping all cases inside the box.
    # Fixture rays use the REPORTED Q14 angle, so only range quantization remains.
    native_deg = (phase_deg+np.arange(COUNT)*360/COUNT) % 360
    q_angles = np.floor(native_deg*16384/90).astype(int)
    angles = math.pi-q_angles*math.pi/32768  # default inverted=false frame convention
    hits, walls, ranges = [], [], []
    for t, a in zip(times, angles):
        p, rot = pose(t-START, v, w)
        origin = p+rot@np.array([CONFIG.lidar_x_m, CONFIG.lidar_y_m])
        direction = rot@np.array([math.cos(a), math.sin(a)])
        choices = []
        for axis in (0, 1):
            if abs(direction[axis]) > 1e-12:
                wall = math.copysign(ROOM[axis], direction[axis])
                distance = (wall-origin[axis])/direction[axis]
                if distance > 0:
                    choices.append((distance, axis))
        distance, axis = min(choices)
        hits.append(origin+distance*direction)
        walls.append(axis)
        ranges.append(distance)
    range_q2 = np.rint(np.array(ranges)*4000).astype(int)
    if missing:
        range_q2[::13] = 0
    return dict(times=times, angles=angles, q_angles=q_angles, range_q2=range_q2,
                hits=np.asarray(hits), wall_axes=np.asarray(walls))


def motion(v, w):
    history = MotionHistory(CONFIG)
    for stamp in np.arange(.9, NOW+.005, .005):
        history.add_mean_wheel(float(stamp), v/CONFIG.wheel_radius_m)
        history.add_gyro(float(stamp), w)
    return history


def reference_points(data, indices, v, w):
    p, rot = pose(NOW-START, v, w)
    return (data['hits'][indices]-p)@rot


def error_metrics(points, expected, axes, w):
    delta = points-expected
    _, rot = pose(NOW-START, 0., w)
    world_delta = delta@rot.T
    distances = np.linalg.norm(delta, axis=1)
    normal = abs(world_delta[np.arange(len(axes)), axes])
    return dict(points=len(points), endpoint_max_m=float(distances.max()),
                endpoint_rms_m=float(np.sqrt(np.mean(distances**2))),
                wall_normal_max_m=float(normal.max()),
                wall_normal_rms_m=float(np.sqrt(np.mean(normal**2))))


def native_scan(probe, data, stamp, duration, out, label):
    text = f'{COUNT} 2 {stamp:.17g} {duration:.17g}\n'+''.join(
        f'{a} {r}\n' for a, r in zip(data['q_angles'], data['range_q2']))
    result = subprocess.run([str(probe)], input=text, capture_output=True, text=True, timeout=10)
    (out/(label+'.input')).write_text(text)
    (out/(label+'.stdout')).write_text(result.stdout)
    (out/(label+'.stderr')).write_text(result.stderr)
    (out/(label+'.exit')).write_text(str(result.returncode)+'\n')
    if result.returncode or result.stderr:
        raise RuntimeError('native pipeline failed: '+label)
    response = json.loads(result.stdout)
    msg = response['measured']
    ranges = np.array([np.inf if r is None else r for r in msg['ranges']])
    labels = np.array([-1 if r is None else round(r)-1
                       for r in response['provenance_labels_only']['ranges']], dtype=int)
    good = np.isfinite(ranges)
    if not np.array_equal(good, labels >= 0) or np.any(labels[good] >= COUNT):
        raise ValueError('diagnostic provenance mismatch')
    np.testing.assert_allclose(ranges[good], data['range_q2'][labels[good]]/4000,
                               rtol=0, atol=1e-6)
    return msg, ranges, labels


def compare_case(probe, out, name, v, w, phase, wait, missing=False):
    data = scene(v, w, phase, missing)
    h = motion(v, w)
    # 100ms wait aligns favourably with first sample; ready-data 1ms wait occurs
    # after acquisition. Both are explicit hypothetical schedules, not C1 measurements.
    stamp, duration = (START, .1) if wait == '100ms' else (1.119, .001)
    msg, ranges, labels = native_scan(probe, data, stamp, duration, out, name)
    valid = np.flatnonzero(np.isfinite(ranges))
    ids = labels[valid]
    expected = reference_points(data, ids, v, w)
    metrics = {}
    for mode, key in [('none', 'vendor_no_motion'), ('both', 'vendor_index_time_assumed')]:
        points, indices, _ = h.project(ranges, msg['angle_min'], msg['angle_increment'],
            .05, 12., msg['stamp_s'], msg['time_increment'], NOW, mode, True)
        np.testing.assert_array_equal(indices, valid)
        metrics[key] = error_metrics(points, expected, data['wall_axes'][ids], w)
    # Oracle-only ablation: restore ORIGINAL angles and times using diagnostic IDs.
    # It does NOT claim these IDs exist in LaserScan or can be recovered in practice.
    unique_ids = np.unique(ids)
    raw_ranges = data['range_q2']/4000.
    raw_ranges[data['range_q2'] == 0] = np.inf
    for offset in (0., -.01, .01):
        points, indices, _ = project_acquisition_points(h, raw_ranges, data['angles'],
            data['times']+offset, NOW, .05, 12., timing_verified=True)
        metrics['raw_explicit_'+str(offset)] = error_metrics(points,
            reference_points(data, indices, v, w), data['wall_axes'][indices], w)
    oracle, _, _ = project_acquisition_points(h, raw_ranges[unique_ids],
        data['angles'][unique_ids], data['times'][unique_ids], NOW, .05, 12., timing_verified=True)
    metrics['surviving_original_oracle'] = error_metrics(oracle,
        reference_points(data, unique_ids, v, w), data['wall_axes'][unique_ids], w)
    # Angle-bin-only error: correct source time, retain vendor's bin angle/range.
    bin_angles = msg['angle_min']+valid*msg['angle_increment']
    local = np.column_stack((ranges[valid]*np.cos(bin_angles)+CONFIG.lidar_x_m,
                             ranges[valid]*np.sin(bin_angles)+CONFIG.lidar_y_m))
    target, target_rot = pose(NOW-START, v, w)
    corrected = []
    for point, i in zip(local, ids):
        p, rot = pose(data['times'][i]-START, v, w)
        corrected.append((p+rot@point-target)@target_rot)
    metrics['vendor_bins_true_time_oracle'] = error_metrics(np.array(corrected), expected,
                                                           data['wall_axes'][ids], w)
    assigned_times = msg['stamp_s']+valid*msg['time_increment']
    result = dict(name=name, speed_m_s=v, yaw_rate_rad_s=w, phase_deg=phase,
        synthetic_wait=wait, missing=missing, input_count=COUNT,
        input_valid_count=int(np.count_nonzero(data['range_q2'])), output_count=len(ranges),
        output_finite_count=len(ids), unique_source_count=len(unique_ids),
        extra_occurrences=len(ids)-len(unique_ids),
        index_time_error_max_s=float(np.max(abs(assigned_times-data['times'][ids]))),
        metrics=metrics)
    checks = dict(raw_geometry_within_q2_rounding=metrics['raw_explicit_0.0']['endpoint_max_m'] <= .000126,
                  surviving_original_within_q2_rounding=metrics['surviving_original_oracle']['endpoint_max_m'] <= .000126,
                  missing_never_filled_as_new_source=bool(np.all(data['range_q2'][ids] > 0)))
    if v == w == 0:
        checks['stationary_motion_does_not_change_points'] = (
            abs(metrics['vendor_no_motion']['endpoint_max_m']-
                metrics['vendor_index_time_assumed']['endpoint_max_m']) < 1e-12)
    result['checks'] = checks
    # Enough to replay without new geometry generation; no measured raw-sensor bag.
    np.savez_compressed(out/(name+'.npz'), **data, emitted_ranges=ranges,
                        diagnostic_source_indices=labels)
    return result


def run(upstream, out):
    out.mkdir(parents=True, exist_ok=False)
    sources = [Path(__file__).resolve(), ROOT/'tests/fixtures/c1_scan_pipeline_stub.cpp',
               ROOT/'tests/test_c1_motion_pipeline.py',
               ROOT/'src/arena_autonomy/arena_autonomy/lidar_motion.py',
               ROOT/'src/arena_autonomy/arena_autonomy/lidar_point_review.py']
    manifest = {str(p.relative_to(ROOT)): digest(p) for p in sources}
    for p in sources:
        dest = out/'source'/p.relative_to(ROOT)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(p, dest)
    (out/'source_manifest.json').write_text(json.dumps(manifest, indent=2))
    probe = build_native(upstream, out)
    cases = []
    for name, v, w in [('stationary', 0., 0.), ('straight', 1., 0.),
                        ('left', 1., 1.), ('right', 1., -1.)]:
        for phase in (0., 93.):
            for wait in ('100ms', '1ms'):
                label = f'{name}_phase{int(phase)}_{wait}'
                cases.append(compare_case(probe, out, label, v, w, phase, wait))
    cases.append(compare_case(probe, out, 'left_missing', 1., 1., 93., '1ms', True))
    unchanged = all(digest(ROOT/name) == sha for name, sha in manifest.items())
    upstream_unchanged = all(digest(upstream/name) == sha for name, sha in PINNED.items())
    passed = unchanged and upstream_unchanged and all(all(c['checks'].values()) for c in cases)
    report = dict(scope='analytic_box_native_selected_functions_type_stubs_no_device_no_ros',
        upstream_commit=COMMIT, upstream_file_hashes=PINNED, case_count=len(cases),
        scan_hz=HZ, samples_per_scan=COUNT, now_s=NOW, source_unchanged=unchanged,
        upstream_unchanged=upstream_unchanged, diagnostic_checks_passed=passed,
        hardware_verified=False, operationally_connected=False, ready_to_drive=False,
        fixture='6 x 3 m analytic box, native reported angles, quarter-mm range rounding; no noise/slip/dropout except one labelled missing case',
        timing='synthetic point clock, ideal 200 Hz mean wheel/gyro; 100ms and 1ms host wait hypotheses; offsets +-10ms are sensitivity probes',
        cases=cases)
    (out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    (out/'exit.txt').write_text(('0' if passed else '1')+'\n')
    print(json.dumps(dict(cases=len(cases), diagnostic_checks_passed=passed,
                         source_unchanged=unchanged, report=str(out/'report.json'))))
    return 0 if passed else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', type=Path, required=True, help='pinned local source cache; never downloaded here')
    parser.add_argument('--output', type=Path, required=True, help='new evidence directory; existing path rejected')
    args = parser.parse_args()
    raise SystemExit(run(args.upstream.resolve(), args.output.resolve()))

"""보존한 정지 스캔과 설치 Ogre2의 최근접 cubemap 표본을 사후 대조한다.

원래 시험의 허용치/판정을 고치거나 센서 값을 수정하지 않는다.
"""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
BASE = ROOT/'artifacts/validation/2026-10-23/bump_lidar_pose_v1/result'
paths = [BASE/'report.json', BASE/'scan_frames.json', BASE/'fixture.sdf']
before = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
original = json.loads(paths[0].read_text())
frames = json.loads(paths[1].read_text())
assert original['passed'] is False and original['geometry_matched'] is False
assert original['shutdown_audit']['clean'] is False

# gz-rendering8_8.2.3 Ogre2GpuRays::ConfigureCamera: 90° 구간의 표본수,
# next-power-of-two 후 128..1024 제한. 500개/360°는 128 texel이다.
horizontal_samples = 500
hfov = 2*math.pi*(horizontal_samples-1)/horizontal_samples
hs = int((math.pi/2)/hfov*horizontal_samples)
texture_size = min(1024, max(128, 1 << (max(hs, 1)-1).bit_length()))
assert texture_size == 128


def raster_range(height, pitch, yaw, size):
    # 전방 cubemap 한 면만. texture filtering none에서 v=.5는 아래 픽셀
    # 중심을 선택한다. 정규화 전 광선은 (1, quantized tan(yaw), -1/size).
    assert abs(yaw) < math.pi/4 and size > 0
    lateral = 2*(math.floor((math.tan(yaw)+1)*size/2)+.5)/size-1
    down = math.sin(pitch)+math.cos(pitch)/size
    return None if down <= 0 else height*math.sqrt(1+lateral*lateral+1/size**2)/down


assert raster_range(.1, 0, 0, 128) > 12
assert raster_range(.1, -.074, 0, 128) is None
assert raster_range(.1, .074, 0, 1024) > raster_range(.1, .074, 0, 128)
rows, notes = [], []
for name, scans in frames.items():
    view = original['views'][name]
    assert len(scans) == 3
    for frame_index, scan in enumerate(scans):
        assert len(scan['ranges']) == 500
        assert abs(scan['angle_min']+math.pi) < 1e-5
        assert abs(scan['angle_increment']-2*math.pi/500) < 1e-6
        assert scan['range_max'] == 12.
        for index, measured in enumerate(scan['ranges']):
            angle = scan['angle_min']+index*scan['angle_increment']
            if abs(angle) > math.radians(10):
                continue
            predicted = raster_range(view['sensor_height_m'], view['sdf_pitch_rad'], angle, texture_size)
            predicted_hit = predicted is not None and predicted < scan['range_max']
            measured_hit = measured is not None and measured < scan['range_max']
            if measured is not None:
                assert math.isfinite(measured) and measured >= scan['range_min']
            residual = abs(measured-predicted) if measured_hit and predicted_hit else None
            rows.append(dict(view=name, frame=frame_index, index=index, measured_m=measured,
                             raster_predicted_m=predicted, expected_hit=predicted_hit,
                             hit_agrees=predicted_hit == measured_hit, residual_m=residual))
assert all(r['hit_agrees'] for r in rows)
residuals = [r['residual_m'] for r in rows if r['residual_m'] is not None]
installed_root = Path('/opt/ros/jazzy/opt/gz_rendering_vendor')
installed = [installed_root/'include/gz/rendering8/gz/rendering/config.hh']
media = installed_root/'share/gz/gz-rendering8/ogre2/media/materials'
installed += [media/'scripts/gpu_rays.material',
              media/'programs/GLSL/gpu_rays_1st_pass_fs.glsl',
              media/'programs/GLSL/gpu_rays_2nd_pass_fs.glsl']
assert '#define GZ_RENDERING_VERSION_FULL "8.2.3"' in installed[0].read_text()
assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == value for p, value in before.items())
result = dict(scope='posthoc_static_front_ray_raster_explanation_not_new_acceptance_test',
    original_passed=original['passed'], original_geometry_matched=original['geometry_matched'],
    original_shutdown_clean=original['shutdown_audit']['clean'], input_hashes=before,
    texture_size=texture_size, vertical_half_pixel_angle_deg=math.degrees(math.atan(1/texture_size)),
    compared=len(rows), finite_compared=len(residuals), maximum_residual_m=max(residuals),
    measured_front_m=frames['nose_down'][0]['ranges'][250],
    installed_source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in installed},
    source_refs=['https://github.com/gazebosim/gz-rendering/blob/gz-rendering8_8.2.3/ogre2/src/Ogre2GpuRays.cc',
                 'installed 8.2.3 gpu_rays.material and GLSL first/second pass shaders'],
    rows=rows, original_failure_preserved=True, sensor_values_modified=False,
    production_configuration_modified=False, additional_gazebo_run=False,
    limits=['Only these saved static front rays and this software renderer setup.',
            'Posthoc mechanism agreement is not a certified global error bound.',
            'Not real C1 accuracy, rotating acquisition, vehicle dynamics, or a correction policy.',
            'Installed shaders are read directly; the C++ reference is the matching upstream version tag, not a binary-source attestation.'])
with (OUT/'report.json').open('x') as handle:
    json.dump(result, handle, indent=2, allow_nan=False)
print(json.dumps({key: result[key] for key in ('compared','finite_compared','texture_size','maximum_residual_m','original_failure_preserved')}))

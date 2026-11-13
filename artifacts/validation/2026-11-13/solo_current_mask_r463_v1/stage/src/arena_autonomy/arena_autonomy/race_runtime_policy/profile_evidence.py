"""부분 장착 성분의 출처 대조. 광선 상태/시각/거리를 생성하거나 수정하지 않는다."""
import math

from arena_autonomy.local_scene import LocalScene, blind_spec, range_bands, _blind_limits, _masked_by
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.observation_support import source_valid
from arena_autonomy.crop_edge_selection import SIMULATION_SCOPE, ESTIMATED_SCOPE, validate_observation_scope, timing_matches


def profile_evidence(scene, raw, sectors, profile, motion_epoch, timing_verified, *,
                     observation_scope=SIMULATION_SCOPE,timing_policy='verified_only'):
    """거리 한정 원 프로필과 검증된 현재 장면의 일치. 전체-sector 경로는 호출하지 않는다."""
    validate_observation_scope(observation_scope)
    valid_timing = (timing_matches(observation_scope,timing_verified,timing_policy,raw.time_basis)
        if observation_scope==ESTIMATED_SCOPE and type(raw) is MeasurementScan
        else timing_verified is True and getattr(raw,'time_basis',None) != 'unverified_estimate')
    if (type(raw) is not MeasurementScan or type(scene) is not LocalScene
            or not valid_timing
            or type(motion_epoch) is not int or motion_epoch < 0
            or type(profile) is not dict or 'range_limits_m' not in profile):
        raise ValueError('explicit_ranged_profile_and_verified_source_required')
    rear, declared = blind_spec(profile)
    bands = range_bands(profile, len(declared))
    limits = _blind_limits(profile)
    if rear != 0. or not declared or tuple(map(tuple, sectors)) != declared:
        raise ValueError('profile_sector_mismatch')
    total = raw.source_epoch + motion_epoch
    epoch = total*(total+1)//2 + motion_epoch
    if (not source_valid(scene) or scene.source_epoch != epoch
            or scene.source_frame_id != raw.source_frame_id
            or scene.source_first_s != raw.acquired_at_s[0]
            or scene.source_last_s != raw.acquired_at_s[-1]
            or scene.source_angles_rad != raw.angles_rad
            or len(scene.rays) != len(raw.ranges_m)):
        raise ValueError('profile_raw_scene_metadata_mismatch')
    for i, (ray, distance, angle, stamp) in enumerate(zip(
            scene.rays, raw.ranges_m, raw.angles_rad, raw.acquired_at_s)):
        valid = math.isfinite(distance) and raw.range_min_m <= distance <= raw.range_max_m
        masked = _masked_by(math.remainder(angle, math.tau), limits, distance)
        status = 'masked' if masked else ('return' if valid else 'unknown')
        if (ray.source_index != i or ray.source_frame_id != raw.source_frame_id
                or ray.source_epoch != epoch or ray.acquired_at_s != stamp or ray.status != status):
            raise ValueError('profile_raw_scene_ray_mismatch')
        if status == 'return':
            hit = tuple(o+distance*d for o, d in zip(ray.ray_origin_xy_m, ray.direction_xy))
            if ray.hit_xy_m is None or math.dist(hit, ray.hit_xy_m) > 1e-6:
                raise ValueError('profile_return_geometry_mismatch')
        elif ray.hit_xy_m is not None:
            raise ValueError('profile_nonreturn_has_hit')
    return bands


def partial_component(masked, memberships, a, b, values, bands):
    """nominal 구간을 실제 먼 return이 끊은 한 연결 성분만 인정한다."""
    matches = [i for i, member in enumerate(memberships) if masked and masked < member]
    if len(matches) != 1:
        return None
    sector = matches[0]
    # 겹치거나 여러 장착 구간의 mask를 한 부분 구간으로 합치지 않는다.
    if any(sum(j in member for member in memberships) != 1 for j in masked):
        return None
    inside_flanks = [j for j in (a, b) if j in memberships[sector]]
    if not inside_flanks or any(not math.isfinite(values[j]) or values[j] <= bands[sector][1]
                                for j in inside_flanks):
        return None
    return dict(sector_index=sector, masked_indices=sorted(masked),
                distant_split_return_indices=inside_flanks,
                range_band_m=list(bands[sector]), unknown_filled=False)

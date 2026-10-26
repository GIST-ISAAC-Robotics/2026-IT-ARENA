"""GPT independent review: hidden whole-vehicle shape and sampled turning contact.

Evaluation only. No controller, sensor configuration, or operating defaults are changed.
Truth polygons generate synthetic rays and evaluate overlap; they are never controller inputs.
The existing fixed-input comparison is not a closed-loop or hardware safety certificate.
"""
import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT / 'src/arena_autonomy'), str(HERE)]
import numpy as np
from arena_autonomy.local_scene import scene_from_scan
import synthetic_world as sw


@dataclass
class PolygonBody:
    local: np.ndarray
    trajectory: object
    name: str = 'hidden_vehicle_shape'

    def polygon(self, t):
        return self.local + np.asarray(self.trajectory(t))


def polygon_intersection(a, b):
    """Convex clipping. A positive area is a sampled geometric overlap witness."""
    result = [np.asarray(p) for p in a]
    for p, q in zip(b, np.roll(b, -1, axis=0)):
        if not result:
            return 0.
        edge = q-p
        def signed(x):
            delta = x-p
            return edge[0]*delta[1]-edge[1]*delta[0]
        old, result = result, []
        for left, right in zip(old, old[1:]+old[:1]):
            dl, dr = signed(left), signed(right)
            if dl >= 0:
                result.append(left)
            if (dl >= 0) != (dr >= 0):
                result.append(left + (right-left)*dl/(dl-dr))
    if len(result) < 3:
        return 0.
    points = np.asarray(result)
    return abs(float(np.sum(points[:, 0]*np.roll(points[:, 1], -1)
                            - points[:, 1]*np.roll(points[:, 0], -1))))/2


def body_polygon(distance, steer, wheelbase=.135):
    k = math.tan(steer)/wheelbase
    yaw = k*distance
    c, s = math.cos(yaw), math.sin(yaw)
    rear_x = distance if not k else s/k
    rear_y = 0. if not k else 2*math.sin(yaw/2)**2/k
    center = np.array([rear_x-wheelbase/2+wheelbase/2*c, rear_y+wheelbase/2*s])
    corners = np.array([[-.10, -.075], [.10, -.075], [.10, .075], [-.10, .075]])
    return corners @ np.array([[c, s], [-s, c]]) + center


def scene_with_mask(world, t, ranges, stamps, half_angle):
    # Fresh synthetic returns; never reconstruct an already-masked stored scan.
    return scene_from_scan(ranges.tolist(), angle_min=-math.pi,
        angle_increment=2*math.pi/sw.N_RAYS, range_min=.05, range_max=12.,
        acquired_at_s=stamps.tolist(), t_ref_s=t,
        acquisition_to_ref_se2=[(world.ego_x(s)-world.ego_x(t), 0., 0.) for s in stamps],
        rear_half_angle_deg=half_angle)


def main(output):
    if output.exists():
        raise FileExistsError(output)
    sources = [Path(__file__), HERE/'synthetic_world.py',
               ROOT/'src/arena_autonomy/arena_autonomy/local_scene.py']
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    # A convex 20 x 15 cm outline with a narrow front tip. Its entire body, not
    # just an isolated millimetre object, can lie in the assumed 60-degree mask.
    gap = .001
    # Keep the full hidden outline below the passing actor's nearest lateral
    # boundary (relative y=.145). The earlier illustrative rear corner at .170
    # could overlap the passing actor; that original result remains archived.
    triangle = np.array([[-.098, .075+gap], [-.298, .140], [-.298, -.010]])
    lengths = np.linalg.norm(triangle-np.roll(triangle, -1, axis=0), axis=1)
    twice_area = abs(float(np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])))
    inradius = twice_area / float(np.sum(lengths))
    world = sw.World(.20, -.09)
    overtaker = sw.Body(.20, .15, lambda t: (.2-.6+.45*(t-1.), .13), 'visible_overtaker')
    hidden = PolygonBody(triangle, lambda t: (world.ego_x(t), world.ego_y))
    angles = [30., 25., 20., 0.]
    rows = []
    actor_overlap_max = 0.
    for tick in range(81):
        t = round(1.+tick*.1, 3)
        base, stamps = world.ranges(t, [overtaker])
        extra, other_stamps = world.ranges(t, [overtaker, hidden])
        actor_overlap_max=max(actor_overlap_max,polygon_intersection(hidden.polygon(t),overtaker.polygon(t)))
        assert np.array_equal(stamps, other_stamps)
        rows.append({'t': t, 'same_scan_by_rear_half_angle_deg': {
            str(angle): sw.same_scan(scene_with_mask(world, t, base, stamps, angle),
                                    scene_with_mask(world, t, extra, stamps, angle))
            for angle in angles}})
    steering_rows = []
    for steer in [-.30, .30, 0.]:
        witnesses = []
        for time in np.arange(0., .301, .001):
            travel = .20*float(time)
            ego = body_polygon(travel, steer)
            other = triangle + [travel, 0.]
            area = polygon_intersection(other, ego)
            if area > 1e-12:
                witnesses.append({'t_s': float(time), 'travel_m': travel,
                                  'overlap_area_m2': area, 'ego_corners': ego.tolist(),
                                  'hidden_polygon': other.tolist()})
        steering_rows.append({'steer_rad': steer, 'sample_overlap_count': len(witnesses),
                              'first_witness': witnesses[0] if witnesses else None})
    checks = {
        'within_registered_20_by_15_cm_envelope': bool(np.ptp(triangle[:, 0]) <= .200000001
                                                     and np.ptp(triangle[:, 1]) <= .150000001),
        'contains_disk_diameter_greater_than_20mm': inradius*2 > .020,
        'all_81_current_mask_scans_identical': all(r['same_scan_by_rear_half_angle_deg']['30.0'] for r in rows),
        'some_scans_distinguishable_without_mask': any(not r['same_scan_by_rear_half_angle_deg']['0.0'] for r in rows),
        'initial_no_overlap': polygon_intersection(triangle, body_polygon(0., 0.)) == 0.,
        'hidden_actor_disjoint_from_overtaker': actor_overlap_max == 0.,
        'lateral_separation_from_overtaker_m': bool(.13-.15/2-(world.ego_y+float(triangle[:,1].max())) > 0),
        'left_turn_has_overlap_witness': bool(steering_rows[1]['sample_overlap_count']),
        'straight_has_no_sample_overlap': not steering_rows[2]['sample_overlap_count'],
    }
    report = {'scope': 'synthetic visibility and sampled kinematic counterexample, no controller/physics trial',
        'decision': 'Do not infer local minimum cross-section from whole-vehicle width or inradius.',
        'hidden_triangle_xy_m': triangle.tolist(), 'initial_lateral_tip_gap_m': gap,
        'other_actor_max_sample_overlap_m2':actor_overlap_max,
        'other_actor_lateral_gap_m':.13-.15/2-(world.ego_y+float(triangle[:,1].max())),
        'whole_shape_incircle_diameter_m': inradius*2, 'visibility_rows': rows,
        'same_scan_counts': {str(a): sum(r['same_scan_by_rear_half_angle_deg'][str(a)] for r in rows) for a in angles},
        'steering': steering_rows, 'checks': checks, 'source_sha256': hashes,
        'source_unchanged': all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in hashes.items()),
        'limits': ['Illustrative 2D lidar-plane shape, not a hardware design or actual competing vehicle.',
                   'Fixed equal forward motion of the hidden shape and sampled bicycle ego turning.',
                   'P-size at every exposed local section is stronger than whole vehicle width or inradius.',
                   'Smaller mask is a diagnostic configuration, not an adopted hardware setting.']}
    output.mkdir(parents=True)
    (output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'checks': checks, 'same_scan_counts': report['same_scan_counts'],
                      'source_unchanged': report['source_unchanged']}, ensure_ascii=False))
    return 0 if all(checks.values()) and report['source_unchanged'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    sys.exit(main(args.output))

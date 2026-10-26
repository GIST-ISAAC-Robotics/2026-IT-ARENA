"""반례의 평가용 도식. 실제 센서 화면이나 Gazebo 사진이 아니다."""
import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from review_geometry import body_polygon, polygon_intersection
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


def main(source, output):
    result = json.loads(source.read_text(encoding='utf-8'))
    if output.exists():
        raise FileExistsError(output)
    shape = np.asarray(result['hidden_triangle_xy_m'])
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    ax = axes[0]
    ax.set_title('Illustrative lidar-plane shape in a rear mask')
    # Mask is nominal reference-time geometry; actual test uses per-ray times.
    sensor = np.array([.06, 0.])
    distance = .55
    ax.add_patch(Polygon([sensor, sensor+[-distance, distance*np.tan(np.pi/6)],
                         sensor+[-distance, -distance*np.tan(np.pi/6)]], color='#a4acc4', alpha=.30))
    ax.add_patch(Polygon(body_polygon(0., 0.), fc='#70b7ee', ec='#17395d', label='Ego 20 x 15 cm'))
    ax.add_patch(Polygon(shape, fc='#edb27b', ec='#8a3d11', label='Hidden outline 20 x 15 cm'))
    ax.scatter(*sensor, color='black', marker='x', label='Lidar reference origin')
    ax.annotate('Forward', (.20, 0.), (.07, -.19), arrowprops={'arrowstyle':'->'})
    ax.set_xlim(-.38, .22); ax.set_ylim(-.22, .24)
    ax.legend(loc='lower left', fontsize=8)
    ax = axes[1]
    ax.set_title('Left-turn witness: same forward speed, different path')
    # The original run predicted a right-turn intersection and failed that check.
    # Preserve that failure; visualize the observed left-turn intersection instead.
    left = next(row for row in result['steering'] if row['steer_rad'] == .30)
    witness = left['first_witness']
    if witness:
        at = .250  # Fixed interior time within the original 0..0.300 s sample interval.
        travel = .20*at
        ego = body_polygon(travel, .30)
        hidden = shape + [travel, 0.]
        area = polygon_intersection(hidden, ego)
        if area <= 1e-9:
            raise ValueError('fixed 0.250 s left-turn sample has no meaningful overlap')
        ax.add_patch(Polygon(ego, fc='#70b7ee', ec='#17395d', alpha=.65))
        ax.add_patch(Polygon(hidden, fc='#edb27b', ec='#8a3d11', alpha=.65))
        ax.set_xlim(-.075, -.015); ax.set_ylim(.055, .100)
        ax.text(.03, .96, f"Fixed sample: {at:.3f} s; overlap {area*1e6:.3f} mm2\n"
                f"Initial tip gap: {1000*result['initial_lateral_tip_gap_m']:.1f} mm\n"
                'Kinematics only; no closed-loop/physics claim', transform=ax.transAxes,
                va='top', fontsize=8)
        output.with_suffix('.json').write_text(json.dumps({
            'source_result': str(source),
            'source_checks': result['checks'],
            'fixed_time_s': at, 'steer_rad': .30, 'overlap_area_m2': area,
            'ego_corners': ego.tolist(), 'hidden_polygon': hidden.tolist()}, indent=2)+'\n', encoding='utf-8')
    for ax in axes:
        ax.set_aspect('equal'); ax.grid(alpha=.2); ax.set_xlabel('Forward x [m]'); ax.set_ylabel('Left y [m]')
    fig.suptitle('Synthetic geometry review — not an actual competing vehicle or installation drawing', fontsize=11)
    fig.tight_layout(); fig.savefig(output, dpi=170); plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    main(args.input, args.output)

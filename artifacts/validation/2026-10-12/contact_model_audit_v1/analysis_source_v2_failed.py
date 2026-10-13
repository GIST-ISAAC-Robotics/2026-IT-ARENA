#!/usr/bin/env python3
"""기존 제동 기록의 구동/접촉 정합성 감사. 런타임·물성·기존 증거를 수정하지 않는다."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
WHEELS = ('front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel')
REAR = WHEELS[2:]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def rotate(v, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    matrix = ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
              (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))
    return [dot(row, v) for row in matrix]


def vec(value):
    v = [float(value.get(k, 0)) for k in ('x', 'y', 'z')]
    if not all(math.isfinite(x) for x in v):
        raise ValueError('nonfinite vector')
    return v


def unit(v):
    length = math.sqrt(dot(v, v))
    if length < 1e-10:
        raise ValueError('degenerate direction')
    return [x/length for x in v]


def friction_components(force, normal, axis, mu):
    """DART friction pyramid directions. A diagonal can exceed a circular cone lawfully."""
    normal = unit(normal)
    t2 = unit(cross(normal, axis))
    t1 = unit(cross(t2, normal))
    fn, ft1, ft2 = dot(force, normal), dot(force, t1), dot(force, t2)
    return {'normal_n': fn, 'lateral_n': ft1, 'longitudinal_n': ft2,
            'pyramid_bound_ratio': max(abs(ft1), abs(ft2))/(mu*fn) if fn > .01 else None}


def read_contacts(path):
    result = {w: {} for w in WHEELS}
    duplicate = 0
    with path.open() as stream:
        for line in stream:
            msg = json.loads(line)
            stamp = msg['header']['stamp']
            ns = int(stamp.get('sec', 0))*10**9+int(stamp.get('nsec', 0))
            grouped = {}
            for contact in msg.get('contact', []):
                names = [contact[f'collision{i}']['name'] for i in (1, 2)]
                for side, name in enumerate(names, 1):
                    wheel = next((w for w in WHEELS if name == f'arena_car::{w}::collision'), None)
                    if not wheel:
                        continue
                    positions, normals, wrenches = (contact.get(k, []) for k in ('position', 'normal', 'wrench'))
                    if not positions or not len(positions) == len(normals) == len(wrenches):
                        raise ValueError('incomplete contact points')
                    for pos, norm, wrench in zip(positions, normals, wrenches):
                        if 'force' not in wrench.get(f'body{side}Wrench', {}):
                            raise ValueError('missing force')
                        grouped.setdefault(wheel, []).append({
                            'position': vec(pos), 'normal': [x*(1 if side == 1 else -1) for x in vec(norm)],
                            'force': vec(wrench[f'body{side}Wrench']['force']), 'other': names[2-side]})
            for wheel, points in grouped.items():
                if ns in result[wheel]:
                    if result[wheel][ns] != points:
                        raise ValueError('conflicting same-time contacts')
                    duplicate += 1
                else:
                    result[wheel][ns] = points
    return result, duplicate


def impulse(records, lo, hi, step_ns=1000000):
    """Post-step forces at (lo, hi]; refuse missing/intermediate timestamps, no zero fill."""
    times = sorted(t for t in records if lo < t <= hi)
    expected = list(range(lo+step_ns, hi+1, step_ns))
    if not expected or times != expected:
        raise ValueError('contact impulse coverage gap')
    return [sum(p['force'][axis] for t in times for p in records[t])*step_ns/1e9 for axis in range(3)]


def model_params(path):
    model = ET.parse(path).find('model')
    if any(p.get('relative_to') for p in model.findall('.//pose')) or any(
            a.get('expressed_in') for a in model.findall('.//axis/xyz')):
        raise ValueError('non-default frame unsupported by this bounded audit')
    plugin = model.find("plugin[@name='arena::SingleMotorDrive']")
    drive = {name: float(plugin.findtext(name)) for name in ('wheel_radius', 'gear_ratio', 'gear_efficiency',
             'motor_brake_torque_limit', 'speed_kp', 'carrier_drag', 'differential_viscosity', 'differential_torque_limit')}
    poses = {link.get('name'): [float(v) for v in link.findtext('pose', '0 0 0 0 0 0').split()] for link in model.findall('link')}
    masses = {link.get('name'): float(link.findtext('inertial/mass')) for link in model.findall('link')}
    mass = sum(masses.values())
    com = [0., 0., 0.]
    for link in model.findall('link'):
        name = link.get('name')
        ipose = [float(v) for v in link.findtext('inertial/pose', '0 0 0 0 0 0').split()]
        offset = rotate(ipose[:3], poses[name][3:])
        for i in range(3):
            com[i] += masses[name]*(poses[name][i]+offset[i]-poses['chassis'][i])/mass
    wheel_info = {}
    for wheel in WHEELS:
        link = model.find(f"link[@name='{wheel}']")
        joint = next(j for j in model.findall('joint') if j.findtext('child') == wheel)
        axis = [float(v) for v in joint.findtext('axis/xyz').split()]
        ode = link.find('collision/surface/friction/ode')
        fdir = [float(v) for v in ode.findtext('fdir1').split()]
        slip = model.find(f"plugin[@name='gz::sim::systems::WheelSlip']/wheel[@link_name='{wheel}']")
        wheel_info[wheel] = {'axis_model': rotate(axis, poses[wheel][3:]),
            'fdir_model': rotate(fdir, poses[wheel][3:]), 'mu': float(ode.findtext('mu')),
            'mu2': float(ode.findtext('mu2')), 'spin_inertia': float(link.findtext('inertial/inertia/izz')),
            'offset_chassis': [poses[wheel][i]-poses['chassis'][i] for i in range(3)],
            'joint_damping': float(joint.findtext('axis/dynamics/damping')),
            'slip_compliance': float(slip.findtext('slip_compliance_longitudinal')),
            'normal_reference_n': float(slip.findtext('wheel_normal_force'))}
        if float(link.findtext('collision/geometry/cylinder/radius')) != drive['wheel_radius']:
            raise ValueError('wheel geometry and drive radius differ')
        if wheel in REAR and joint.findtext('parent') != 'chassis':
            raise ValueError('unsupported rear joint parent')
        if wheel_info[wheel]['mu'] != wheel_info[wheel]['mu2']:
            raise ValueError('audit currently requires equal mu/mu2')
    return {'drive': drive, 'wheels': wheel_info, 'mass_kg': mass, 'com_from_chassis_m': com}


def exact_sample(row, contacts, params):
    ns = round(row['sim_time_s']*1e9)
    rpy = [row[f'truth_{a}_rad'] for a in ('roll', 'pitch', 'yaw')]
    output = {'sim_time_s': row['sim_time_s'], 'body_mps': row['truth_longitudinal_mps'],
              'mean_wheel_mps': row['wheel_surface_speed_mps'], 'motor_torque_nm': row['motor_torque_nm'], 'rear': {}}
    for wheel, speed_key, torque_key in zip(REAR, ('left_speed_rad_s', 'right_speed_rad_s'), ('left_torque_nm', 'right_torque_nm')):
        if ns not in contacts[wheel]:
            raise ValueError('no exact same-step contact')
        p = params['wheels'][wheel]
        axis = rotate(p['axis_model'], rpy)
        samples = [friction_components(v['force'], v['normal'], axis, p['mu']) for v in contacts[wheel][ns]]
        vwheel = row[speed_key]*params['drive']['wheel_radius']
        scale = p['slip_compliance']*abs(vwheel)/p['normal_reference_n']
        output['rear'][wheel] = {'wheel_mps': vwheel, 'force_longitudinal_n': sum(s['longitudinal_n'] for s in samples),
            'normal_n': sum(s['normal_n'] for s in samples), 'point_bound_ratio_max': max(s['pyramid_bound_ratio'] or 0 for s in samples),
            'drive_torque_nm': row[torque_key],
            'contact_points': len(samples), 'low_normal_points_excluded_from_ratio': sum(s['pyramid_bound_ratio'] is None for s in samples),
            'expected_unsaturated_slip_mps': scale*abs(sum(s['longitudinal_n'] for s in samples)),
            'wheel_slip_cmd_m_s_per_n_from_post_speed': scale}
    return output


def planar_com_velocity(row, com):
    # Vertical velocity and roll/pitch rate are not recorded; only a near-level XY diagnostic.
    yaw = row['truth_yaw_rad']
    return rotate([row['truth_longitudinal_mps']-row['truth_yaw_rate_rad_s']*com[1],
                   row['truth_lateral_mps']+row['truth_yaw_rate_rad_s']*com[0], 0.], (0., 0., yaw))


def analyze_run(folder, zero):
    report = json.loads((folder/'report.json').read_text())
    for name in ('wheel_contacts', 'braking_diagnostics'):
        if sha(folder/(name+'.jsonl')) != report[name]['sha256']:
            raise ValueError('raw hash mismatch: '+name)
    data = [json.loads(line) for line in (folder/'braking_diagnostics.jsonl').read_text().splitlines()]
    contacts, duplicates = read_contacts(folder/'wheel_contacts.jsonl')
    params = model_params(folder/'runtime_vehicle.sdf')
    radius, ratio = params['drive']['wheel_radius'], params['drive']['gear_ratio']
    checks = {'ratio_radius_preserved': ratio == 15 and radius == .0325,
              'ideal_open_drive': params['drive']['gear_efficiency'] == 1 and params['drive']['carrier_drag'] == 0
              and params['drive']['differential_viscosity'] == params['drive']['differential_torque_limit'] == 0,
              'axes_match': all(abs(dot(p['axis_model'], p['fdir_model'])-1) < 1e-10
                and all(abs(a-b) < 1e-10 for a, b in zip(p['axis_model'], [0, 1, 0])) for p in params['wheels'].values())}
    torque_error = max(abs(r[k]-r['motor_torque_nm']*ratio/2) for r in data for k in ('left_torque_nm', 'right_torque_nm'))
    speed_error = max(abs(r['motor_speed_rad_s']-ratio*(r['left_speed_rad_s']+r['right_speed_rad_s'])/2) for r in data)
    power_error = max(abs(r['pre_motor_power_w']-r['motor_torque_nm']*r['pre_motor_speed_rad_s']) for r in data)
    selected = [r for r in data if zero <= r['sim_time_s'] <= zero+.22]
    if not selected or max(abs(r[f'truth_{a}_rad']) for r in selected for a in ('roll', 'pitch')) > 1e-5:
        raise ValueError('near-planar impulse audit precondition not met')
    timeline = [exact_sample(r, contacts, params) for r in selected]
    # Select a recorded settled-braking interval; do not interpolate sparse torque diagnostics.
    begin = next(r for r in data if r['sim_time_s'] >= zero+.02)
    end = next(r for r in data if r['sim_time_s'] >= zero+.06)
    lo, hi = (round(r['sim_time_s']*1e9) for r in (begin, end))
    wheel_impulses = {w: impulse(contacts[w], lo, hi) for w in WHEELS}
    total = [sum(v[i] for v in wheel_impulses.values()) for i in range(3)]
    va, vb = (planar_com_velocity(r, params['com_from_chassis_m']) for r in (begin, end))
    delta_p = [(b-a)*params['mass_kg'] for a, b in zip(va, vb)]
    residual = [total[i]-delta_p[i] for i in range(2)]
    resting = min(data, key=lambda r: abs(r['sim_time_s']-(report['command_start_sim_s']-.2)))
    static = exact_sample(resting, contacts, params)
    rear = params['wheels'][REAR[0]]
    damping = params['drive']['speed_kp']*radius*ratio/2
    return {'checks': checks, 'parameters': params, 'static_reference': static,
            'post_zero_recorded_point_bound_ratio_max': max(p['point_bound_ratio_max'] for r in timeline for p in r['rear'].values()),
            'gear_torque_max_residual_nm': torque_error, 'motor_speed_max_residual_rad_s': speed_error,
            'same_step_motor_power_max_residual_w': power_error,
            'all_zero_positive_work_j': max(r['zero_positive_work_j'] for r in data),
            'passive_per_wheel_damping_nm_s': damping, 'unloaded_spin_time_constant_s': rear['spin_inertia']/damping,
            'max_each_wheel_braking_torque_nm': params['drive']['motor_brake_torque_limit']*ratio/2,
            'timeline': timeline, 'planar_impulse': {'window_s': [lo/1e9, hi/1e9], 'contact_impulse_ns': total,
              'approx_com_delta_p_kg_m_s': delta_p[:2], 'xy_residual_ns': residual,
              'xy_relative_error': math.hypot(*residual)/math.hypot(*delta_p[:2]),
              'max_abs_roll_pitch_rad': max(abs(r[f'truth_{a}_rad']) for r in selected for a in ('roll', 'pitch'))},
            'identical_duplicates_removed': duplicates,
            'input_sha256': {p: sha(folder/p) for p in ('report.json', 'runtime_vehicle.sdf', 'wheel_contacts.jsonl', 'braking_diagnostics.jsonl')}}


def analyze(base):
    parent = base.parent/'scheduled_surface_v1'
    manifest = json.loads((parent/'final_manifest.json').read_text())
    comparison = json.loads((parent/'analysis_final.json').read_text())
    if not comparison['surface_comparison_valid'] or not manifest['passed']:
        raise ValueError('prior comparison invalid')
    required = ['analysis_final.json']+[n+'/'+f for n in ('s1_official', 'f1_flat') for f in ('report.json', 'runtime_vehicle.sdf')]
    if not all(sha(parent/p) == manifest['artifact_sha256'][p] for p in required):
        raise ValueError('parent evidence changed')
    upstream = json.loads((base/'upstream_sources.json').read_text())
    if not all(sha(base/'upstream'/n) == v['sha256'] for n, v in upstream.items()):
        raise ValueError('primary source capture changed')
    extra = json.loads((base/'upstream_world_source.json').read_text())
    if sha(base/'upstream'/'World.cpp') != extra['sha256']:
        raise ValueError('additional primary source changed')
    runs = {n: analyze_run(parent/n, comparison['contacts'][n]['zero_sim_s']) for n in ('s1_official', 'f1_flat')}
    checks = {n: all(r['checks'].values()) and r['gear_torque_max_residual_nm'] < 1e-9
              and r['motor_speed_max_residual_rad_s'] < 1e-8 and r['same_step_motor_power_max_residual_w'] < 1e-8 for n, r in runs.items()}
    return {'revision': 'v2_no_mixed_time_contact_moment', 'analysis_integrity_passed': all(checks.values()), 'checks': checks, 'runs': runs,
            'source_sha256': {str(Path(__file__).relative_to(ROOT)): sha(Path(__file__))},
            'parent_manifest_sha256': sha(parent/'final_manifest.json'),
            'limits': ['Post hoc audit, not a fresh simulation or a tire-model validation.',
                       'Slip command inferred from post-step speed; actual plugin uses previous JointVelocity.',
                       'Contact points precede position integration; post-pose lever arms cannot prove spin torque balance.',
                       'Saturated friction is not governed by the small-slip linear relation.',
                       'XY momentum is a near-planar diagnostic, not exact full 3D energy conservation.',
                       'Pointwise bound excess is recorded, not hidden in the integrity verdict.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.base.resolve())
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({'checks': result['checks'], 'runs': {k: {'planar_impulse': v['planar_impulse'],
          'peak': max(v['timeline'], key=lambda r: abs(r['mean_wheel_mps']-r['body_mps']))} for k, v in result['runs'].items()}}, indent=2))
    raise SystemExit(0 if result['analysis_integrity_passed'] else 1)

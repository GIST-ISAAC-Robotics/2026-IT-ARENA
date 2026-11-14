"""저장 입력의 두 벽과 짧은 경계 계산 비교. 공통 변경·출력 없음."""
exec((__import__('pathlib').Path(__file__).resolve().parent/'diagnose.py').read_text())
import inspect
import arena_autonomy.local_path as local_path_module

namespace = vars(local_path_module).copy()
exec(inspect.getsource(original_path).replace('>= .28:', '>= .10:'), namespace)
short_path = namespace['local_path']
comparisons = []
for case in ('last_positive', 'first_zero_candidate'):
    data, scan, history = restore_call(json.loads((P/f'{case}_compute_input.json').read_text()), motion)
    scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
        epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'],
        timing_policy=data.get('timing_policy', 'verified_only'))
    for side in ('left', 'right'):
        for mode, path_function in [('original', original_path), ('short_diagnostic_only', short_path)]:
            row = dict(case=case, side=side, mode=mode)
            def traced_path(*args, **kwargs):
                p, b = path_function(*args, **kwargs)
                row['path'] = None if p is None else p.tolist(); row['boundary'] = freeze(b)
                return p, b
            candidate.local_path = traced_path
            output = candidate.current_wall_candidate(scene, data['speed_mps'], side,
                data['previous_steering_rad'], data['dt_s'], owner.config)
            row['output'] = freeze(output)
            comparisons.append(row)
            print(case, side, mode, None if output is None else output[:2], row['boundary'])
candidate.local_path = original_path
(A/'alternatives.json').write_text(json.dumps(freeze(comparisons), indent=2, allow_nan=False), encoding='utf8')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, axes = plt.subplots(1, 2, figsize=(12, 6))
for axis, case in zip(axes, ('last_positive', 'first_zero_candidate')):
    saved = next(r for r in results if r['case'] == case and r['phase'] == 'compute')
    points = np.asarray(saved['occupied_points'])
    axis.scatter(points[:,0], points[:,1], s=8, color='grey', label='Measured occupancy')
    for row in comparisons:
        if row['case'] == case and row['path'] is not None:
            p = np.asarray(row['path'])
            axis.plot(p[:,0], p[:,1], label=row['side']+' '+row['mode'])
    axis.scatter([0],[0],c='black',marker='^',s=80); axis.set_xlim(-.7,1.3);axis.set_ylim(-1.3,1.3)
    axis.set_title(case); axis.set_aspect('equal'); axis.grid();axis.legend(fontsize=7)
fig.tight_layout();fig.savefig(A/'current_geometry.png',dpi=140)

from pathlib import Path
import subprocess,sys,time,json
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
tests=['test_masked_motion_speed','test_pass_release_trial','test_guard_curve_trial',
    'test_traffic_recovery_contract','test_same_path_speed_recheck','test_released_rear_sweep',
    'test_released_fringe_budget','test_parallel_measured_continuity','test_parallel_side_departure',
    'test_source_provenance_r0_plus_b3']
start=time.perf_counter()
with (A/'regression.log').open('w') as log:
    r=subprocess.run([sys.executable,'-B','-m','pytest','-q',*[f'tests/{n}.py' for n in tests]],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
(A/'regression.exit').write_text(str(r.returncode)+'\n')
(A/'regression_result.json').write_text(json.dumps(dict(exit=r.returncode,tests=tests,wall_s=time.perf_counter()-start),indent=2)+'\n')
print('\n'.join((A/'regression.log').read_text().splitlines()[-12:]),flush=True)
raise SystemExit(r.returncode)

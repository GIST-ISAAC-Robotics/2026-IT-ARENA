from pathlib import Path
import json,subprocess,sys,time
A=Path(__file__).resolve().parent
names=['test_corner_boundary_support','test_fragmented_wall_hypothesis','test_local_follow','test_local_follow_validation',
       'test_traffic_boundary_refresh','test_piecewise_wall','test_pass_release_trial','test_bypass_handoff',
       'test_source_provenance_r0_plus_b3']
t=time.perf_counter()
with (A/'regression.log').open('w') as f:
    r=subprocess.run([sys.executable,'-B','-m','pytest','-q',*[f'tests/{n}.py' for n in names]],stdout=f,stderr=subprocess.STDOUT)
(A/'regression.exit').write_text(str(r.returncode)+'\n')
(A/'regression_result.json').write_text(json.dumps(dict(exit=r.returncode,tests=names,wall_s=time.perf_counter()-t),indent=2)+'\n')
print((A/'regression.log').read_text()[-4000:],flush=True)
raise SystemExit(r.returncode)

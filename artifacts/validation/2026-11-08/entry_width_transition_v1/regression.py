from pathlib import Path
import json,subprocess,sys,time
A=Path(__file__).resolve().parent
names=['test_adaptive_corridor_offset','test_short_entry_transition','test_adaptive_entry_preview','test_stopped_entry_centering','test_bypass_entry_trial','test_bypass_entry_guide','test_bypass_entry_follow','test_bypass_handoff','test_bypass_corridor_trial','test_bypass_handoff_corridor_trial','test_source_provenance_r0_plus_b3']
t=time.perf_counter()
with (A/'regression.log').open('w') as f:r=subprocess.run([sys.executable,'-B','-m','pytest','-q',*[f'tests/{n}.py' for n in names]],stdout=f,stderr=subprocess.STDOUT)
(A/'regression.exit').write_text(str(r.returncode)+'\n')
(A/'regression_result.json').write_text(json.dumps(dict(exit=r.returncode,tests=names,wall_s=time.perf_counter()-t),indent=2)+'\n')
print((A/'regression.log').read_text()[-3500:],flush=True)
raise SystemExit(r.returncode)

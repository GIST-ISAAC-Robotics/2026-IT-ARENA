"""Prepare read-only r46 transition replay from the preserved r45 audit."""
from pathlib import Path

A = Path(__file__).resolve().parent
OLD = A.parent / 'corner_r45_transition_audit_v1'
assert not (A / 'capture.py').exists()
s = (OLD / 'capture.py').read_text(encoding='utf8')
s = s.replace('r45', 'r46').replace('corner_occlusion_connected_r46_v1', 'retained_corner_agreement_r46_v1')
s = s.replace("    out = base_original(self, scene, history, *args, **kw)",
    "    prior = dict(wall=self.path_wall, source=self.path_source, reference=self.path_ref, "
    "age=scene.t_ref_s-self.path_source, present=self.path is not None)\n"
    "    out = base_original(self, scene, history, *args, **kw)")
s = s.replace('baseline=compact(out), rays=', 'prior=prior, baseline=compact(out), rays=')
(A / 'capture.py').write_text(s, encoding='utf8')
s = (OLD / 'path_compare.py').read_text(encoding='utf8')
s = s.replace('corner_occlusion_connected_r45_v1', 'retained_corner_agreement_r46_v1')
s = s.replace('for t in (36.52,36.62,36.72,36.82,37.42,37.52,37.62,38.12):',
    'for t in (36.52,36.82,37.12,37.22,37.32,37.42,37.52,37.62,37.72,38.12,38.42,38.52,38.62):')
s = s.replace('reference_present=ref is not None,candidates=[]',
    "reference_present=ref is not None,prior=q['prior'],baseline=q['baseline'],candidates=[]")
s = s[:s.index('print(json.dumps(')] + "print('Saved path_comparison.json:',len(results),'scans')\n"
(A / 'path_compare.py').write_text(s, encoding='utf8')
for name in ('capture.sh', 'path_compare.sh'):
    s = (OLD / name).read_text(encoding='utf8').replace('corner_r45_transition_audit_v1', A.name)
    (A / name).write_bytes(s.replace('\r\n','\n').encode('utf8'))
print('Prepared capture and candidate comparison; production source unchanged.')

"""Prepare exact read-only replay instrumentation for the completed slow actor trial."""
from pathlib import Path

A = Path(__file__).resolve().parent
OLD = A.parent / 'corner_r46_transition_audit_v1'
assert not (A / 'capture.py').exists()
text = (OLD / 'capture.py').read_text(encoding='utf8')
text = text.replace('retained_corner_agreement_r46_v1', 'corner_slowing_actor_v1')
text = text.replace('r46', 'slowing actor')
text = text.replace("'corridor_candidate','parallel_hold_trial')", "'corridor_candidate','parallel_hold_trial','fresh_boundary_accepted',\n    'boundary_candidate_ahead_m','corner_distance_evidence','corner_distance_agrees_memory',\n    'fragmented_boundary_evidence','fragmented_agrees_memory','bilateral_reference')")
(A / 'capture.py').write_text(text, encoding='utf8')
text = (OLD / 'path_compare.py').read_text(encoding='utf8')
text = text.replace('retained_corner_agreement_r46_v1', 'corner_slowing_actor_v1')
begin = text.index('for t in (')
end = text.index("    p=np.asarray", begin)
text = text[:begin] + "for q in captures:\n    if not 34.8 <= q['t'] <= 43.8: continue\n    t=q['t']\n" + text[end:]
(A / 'path_compare.py').write_text(text, encoding='utf8')
for name in ('capture.sh', 'path_compare.sh'):
    text = (OLD / name).read_text(encoding='utf8').replace(OLD.name, A.name)
    text = text.replace('capture_v2', 'capture_v1')
    (A / name).write_text(text, encoding='utf8', newline='\n')
print('Prepared read-only capture; no controller/threshold changes.')

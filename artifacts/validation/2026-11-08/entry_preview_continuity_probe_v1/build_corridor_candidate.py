"""현재 파일의 별도 사본. 적용/시뮬 실행은 하지 않는다."""
from pathlib import Path
import ast,hashlib,json
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
p=ROOT/'src/arena_autonomy/arena_autonomy/bypass_corridor_trial.py';before=p.read_bytes();s=before.decode('utf8')
old="""        return dict(rows=rows, blocked=blocked, chosen=choose_side(rows) if blocked else None,
                    side_comparison_is_permission=False)
"""
new="""        chosen = choose_side(rows) if blocked else None
        result = dict(rows=rows, blocked=blocked, chosen=chosen, side_comparison_is_permission=False)
        if (getattr(self, 'adaptive_preview', False) and chosen is not None
                and rows[chosen]['transition_limit_m'] < min(self.registered_entry.transitions_m)):
            from arena_autonomy.short_entry_transition import compare_short_transition
            candidate = compare_short_transition(scene, self.config, self.registered_entry,
                rows[chosen], evidence['walls'][chosen])
            result['short_transition_comparison'] = candidate
            if candidate['applied']:
                rows[chosen]['transition_m'] = candidate['transition_m']
        return result
"""
assert s.count(old)==1;s=s.replace(old,new);ast.parse(s)
out=A/'bypass_corridor_candidate.py';out.write_bytes(s.encode('utf8'))
(A/'corridor_candidate_manifest.json').write_text(json.dumps(dict(source=p.relative_to(ROOT).as_posix(),before_sha256=hashlib.sha256(before).hexdigest(),
    after_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),helper_sha256=hashlib.sha256((A/'short_transition_candidate.py').read_bytes()).hexdigest()),indent=2)+'\n')
print('corridor candidate copy only')

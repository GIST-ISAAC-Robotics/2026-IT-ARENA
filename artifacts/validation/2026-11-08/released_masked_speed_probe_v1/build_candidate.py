"""현행 선택기의 별도 사본만 생성한다. 실제 src 변경 없음."""
from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
p=ROOT/'src/arena_autonomy/arena_autonomy/pass_release_trial.py';before=p.read_bytes()
text=before.decode('utf8')
needle="        detail['selector_region'] = region\n"
assert text.count(needle)==1
addition='''        if self.follow_config.traffic_recovery_trial and not retained:
            from arena_autonomy.masked_motion_speed import lower_masked_motion
            def evaluate_reduced_target(target):
                reduced_context = {}
                reduced_region = motion_region(scene, speed, target, b_steer, previous, dt,
                    self.follow_config, self.lat, context=reduced_context)
                return reduced_region, reduced_context
            reduced = lower_masked_motion(region, partial, b_speed, self.config, evaluate_reduced_target)
            if reduced is not None:
                detail['selector_masked_speed_recheck'] = reduced['diagnostic']
                if reduced['accepted']:
                    b_speed, region, context = reduced['target_mps'], reduced['region'], reduced['context']
'''
text=text.replace(needle,addition+needle)
needle2="                'selector_reason':'released_bounded_rear_sweep' if fringe.get('policy')"
assert text.count(needle2)==1
text=text.replace(needle2,"                'selector_reason':'released_masked_motion_speed_limited' if detail.get('selector_masked_speed_recheck',{}).get('applied') else\n                                  'released_bounded_rear_sweep' if fringe.get('policy')")
out=A/'pass_release_candidate.py';out.write_bytes(text.encode('utf8'))
(A/'candidate_manifest.json').write_text(json.dumps(dict(source=str(p.relative_to(ROOT)).replace('\\','/'),
    before_sha256=hashlib.sha256(before).hexdigest(),after_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),
    helper_sha256=hashlib.sha256((A/'masked_motion_speed.py').read_bytes()).hexdigest()),indent=2)+'\n')
print('candidate copy only')

"""실행 소스는 보존하고 경계 선택의 마지막 후보만 별도 사본에 연결한다."""
from pathlib import Path
import ast
import hashlib
import json

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
source = ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py'
before = source.read_bytes()
text = before.decode('utf8')
old = """                if eligible:
                    confirmed = carried is not None
"""
new = """                if (not eligible and cfg.traffic_boundary_refresh_trial and cfg.piecewise_wall_trial
                        and carried is not None and self.path_wall in ('left', 'right')):
                    from arena_autonomy.corner_boundary_support import propose
                    supported_corner, support_evidence = propose(points, carried, self.path_wall,
                                                                  cfg, now-self.path_source)
                    detail['current_corner_support'] = {k: v for k, v in support_evidence.items()
                        if not k.endswith('_indices')}
                    if supported_corner is not None:
                        candidate, candidate_wall, eligible = supported_corner, self.path_wall, True
                        # 두 관측 면 사이 교차점은 경로 가설이다. 미관측 길이를 관측 길이로 기록하지 않는다.
                        info = dict(reason='local_path', wall_model='current_two_face_corner',
                            boundary_observed_ahead_m=None,
                            path_anchor_error_m=support_evidence['path_anchor_error_m'])
                        detail.update(boundary_selection='current_two_face_corner_support',
                            boundary_candidate_reason=info['reason'], boundary_candidate_ahead_m=None,
                            corner_supported_path_ahead_m=support_evidence['path_ahead_m'])
                if eligible:
                    confirmed = carried is not None
"""
assert text.count(old) == 1
text = text.replace(old, new)
ast.parse(text)
dest = A/'local_follow_candidate.py'
dest.write_bytes(text.encode('utf8'))
(A/'stage_manifest.json').write_text(json.dumps(dict(
    source=source.relative_to(ROOT).as_posix(),
    before_sha256=hashlib.sha256(before).hexdigest(),
    after_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),
    helper_sha256=hashlib.sha256((A/'candidate.py').read_bytes()).hexdigest(),
    scope='artifact-only staged final fallback; no runtime edits'), indent=2)+'\n')
print('artifact stage created; runtime unchanged')

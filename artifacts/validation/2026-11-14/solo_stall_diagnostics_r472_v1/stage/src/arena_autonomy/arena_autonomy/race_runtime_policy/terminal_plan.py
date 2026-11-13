"""저장 입력 전용: 저속 재획득의 긴 현재 반대 벽 후보. 기본/ROS 미연결."""
import inspect,textwrap
from arena_autonomy import local_follow as lf

ORIGINAL=lf.LocalFollow._baseline_command
INSERT="""                if (not eligible and cfg.standing_wall_reacquire and abs(speed) <= .10
                        and carried is None and preferred is None):
                    # 기존 단일 긴 선호 벽의 저속 재획득과 같은 기준을 반대 벽에도 적용한다.
                    # 선호 벽에 경쟁 경로가 보이면 전환하지 않으며 점유/관측/재개 판정은 뒤에 남긴다.
                    opposite, other = opposite_wall()
                    if sufficient(opposite, other):
                        candidate, info, candidate_wall, eligible = opposite, other, other_side, True
                        detail['boundary_selection'] = 'standing_observed_opposite_wall'
                        detail['boundary_candidate_reason'] = info['reason']
                        detail['boundary_candidate_ahead_m'] = info.get('boundary_observed_ahead_m')
                        detail['single_wall_reacquire'] = {'side': other_side, 'unknown_filled': False,
                            'fresh_observation': True, 'preferred_path_present': False}
"""


def install():
    source=inspect.getsource(ORIGINAL)
    anchor="                if not eligible and cfg.boundary_evidence_mode in ('fragmented_candidate', 'corner_anchor_candidate', 'corner_distance_candidate'):"
    if source.count(anchor)!=1:raise ValueError('candidate source anchor changed')
    namespace={}
    exec(compile(textwrap.dedent(source.replace(anchor,INSERT+anchor)),__file__,'exec'),lf.__dict__,namespace)
    lf.LocalFollow._baseline_command=namespace['_baseline_command']


def uninstall():lf.LocalFollow._baseline_command=ORIGINAL

"""후속 계약 사슬의 기대 부모. 대부분 직전 계약이고, 실제 기록된 분기만 정확한 이름쌍으로 둔다."""

# continuous_prefix_r1은 g2_relative_yield_r2가 아니라 rotation_reobservation_r1에 이어 등록됐다(10/31 기록).
BRANCH_PARENTS = {'config/tests/continuous_prefix_r1_sha256.json': 'config/tests/rotation_reobservation_r1_sha256.json'}


def expected_parent(name, previous):
    return BRANCH_PARENTS.get(name, previous)

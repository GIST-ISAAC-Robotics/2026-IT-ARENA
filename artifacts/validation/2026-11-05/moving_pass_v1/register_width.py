"""중앙 실패 뒤의 폭 선정 후보. 실패한 중단 시안은 소스 체인의 채택판이 아니다."""
from register_witness import create

if __name__=='__main__':
    create('closed_loop_v3','closed_loop_side_follow_1','abort_before.zip',
        'config/tests/moving_pass_witness_r10_sha256.json','config/tests/moving_pass_width_r11_sha256.json',
        {'src/arena_autonomy/arena_autonomy/moving_pass.py':
         '진입 전 두 바깥 벽과 앞면, 순차 스캔 양쪽 나이로 나란히 달릴 폭을 산출해 편이를 고른다. 좁은 경우 기존 추종 유지.'},
        '별도 직선 저속 이동차 후보의 진입 폭 비교. 기본 운용·실물 승인 아님.')

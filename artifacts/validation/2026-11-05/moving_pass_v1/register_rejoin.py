"""복귀 중 뒤에서 접촉한 r11과 현재 차체 중심/전방 관측을 쓰는 r12의 소스 경계."""
from register_witness import create

if __name__=='__main__':
    base='src/arena_autonomy/arena_autonomy/'
    create('closed_loop_side_pass_1','closed_loop_side_pass_2','rejoin_before.zip',
        'config/tests/moving_pass_width_r11_sha256.json','config/tests/moving_pass_rejoin_r12_sha256.json',
        {base+'bypass_entry_trial.py':'기존 후륜축부터 30 cm 앞까지의 벽 범위 검사를 메서드로 분리. 기본 동작 동일.',
         base+'moving_pass.py':'이미 복귀 중이며 상대가 현재 후방에서 충분히 떨어졌을 때 차체 중심부터 전방까지 관측된 긴 벽을 사용. 실제 범위 밖의 경로·점유 완화 없음.'},
        '직선의 저속 이동차 추월 복귀 단계에 한정. 기본 운용·실물 승인 아님.')

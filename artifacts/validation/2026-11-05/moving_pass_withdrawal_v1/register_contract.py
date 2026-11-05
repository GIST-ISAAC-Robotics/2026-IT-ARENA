"""r12 원형 가속 상대와 r13 앞차 뒤 복귀 후보를 같은 source 계약 계보에 잇는다."""
import sys
from prepare import HERE
sys.path.insert(0,str(HERE.parent/'moving_pass_v1'))
from register_witness import create

if __name__=='__main__':
    base='src/arena_autonomy/arena_autonomy/'
    create('../moving_pass_withdrawal_v1/baseline_r12','../moving_pass_withdrawal_v1/candidate_r13',
        '../moving_pass_withdrawal_v1/before_r13.zip','config/tests/moving_pass_rejoin_r12_sha256.json',
        'config/tests/moving_pass_withdrawal_r13_sha256.json',
        {base+'moving_pass.py':'실제 대응 반환 5회 운동 보정 속도 상한과 3회 지속 확인. 차체 전체가 앞이면 현재 복귀 띠로 뒤에 돌아가며 완료와 포기 구분.',
         base+'bypass_entry_trial.py':'기존 복귀 확인 갱신을 메서드로 분리. 일반 우회 기본 동작은 동일.'},
        '직선에서 진입 뒤 속도 이점을 잃은 경우의 선택형 복귀. 기본 운용·실물 승인 아님.')

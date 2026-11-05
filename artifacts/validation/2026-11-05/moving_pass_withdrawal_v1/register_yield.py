"""가속 상대를 계속 따라붙던 r13과 실제 경로에서 양보하는 r14를 분리한다."""
import sys
from prepare import HERE
sys.path.insert(0,str(HERE.parent/'moving_pass_v1'))
from register_witness import create

if __name__=='__main__':
    create('../moving_pass_withdrawal_v1/candidate_r13','../moving_pass_withdrawal_v1/candidate_r14',
        '../moving_pass_withdrawal_v1/before_r14.zip','config/tests/moving_pass_withdrawal_r13_sha256.json',
        'config/tests/moving_pass_yield_r14_sha256.json',
        {'src/arena_autonomy/arena_autonomy/moving_pass.py':
         '양보 의도와 복귀 허가 분리. 상대 후단 하계가 중심보다 앞일 때만 속도 손실 확인; 기존 옆 경로 속도를 완만하게 낮춰 앞차 뒤 공간을 만든 뒤 현재 띠로 복귀.'},
        '직선 저속 추월 중 상대 가속에 대한 선택형 양보·뒤 합류. 실물/일반 추월 승인 아님.')

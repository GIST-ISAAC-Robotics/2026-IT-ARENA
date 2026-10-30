"""전이 길이 관측 선택과 안쪽 경계 수정 뒤 재비교. 시드 고정, 20초 일정."""
from matrix import main

A=dict(surface=True,plan=True,onset=False)
B=dict(surface=True,plan=True,onset=True)
cases=[dict(name=f'A_s{s}',policy=A,seed=s) for s in range(17,23)]+[dict(name='A_ideal',policy=A,sensor='ideal')]
cases+=[dict(name=f'B_s{s}',policy=B,seed=s) for s in range(17,23)]+[dict(name='B_ideal',policy=B,sensor='ideal')]
if __name__=='__main__':
    main('batch2',cases)

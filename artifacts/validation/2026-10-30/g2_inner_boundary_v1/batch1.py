"""요소별 비교. 사전 등록: 시작 가속 3 m/s^2, 20초 일정, 시드는 아래 고정."""
from matrix import main

A=dict(surface=True,plan=True,onset=False)
B=dict(surface=True,plan=True,onset=True)
cases=[dict(name='existing_s17',policy=None),
    dict(name='surface_only_s17',policy=dict(surface=True,plan=False,onset=False)),
    dict(name='plan_only_s17',policy=dict(surface=False,plan=True,onset=False)),
    dict(name='A_s17',policy=A),dict(name='A_s18',policy=A,seed=18),dict(name='A_s19',policy=A,seed=19),
    dict(name='A_s20',policy=A,seed=20),dict(name='A_ideal',policy=A,sensor='ideal'),
    dict(name='B_s17',policy=B),dict(name='B_s18',policy=B,seed=18),dict(name='B_s19',policy=B,seed=19),
    dict(name='B_s20',policy=B,seed=20),dict(name='B_ideal',policy=B,sensor='ideal')]
if __name__=='__main__':
    main('batch1',cases)

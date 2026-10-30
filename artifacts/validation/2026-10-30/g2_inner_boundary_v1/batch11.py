"""최종 코드에서 재확인이 빠진 대조만 실행한다. 수치 변경 없음."""
from matrix import main

B=dict(surface=True,plan=True,onset=True)
cases=[dict(name='edge_left',policy=B,target_y=.15),dict(name='edge_right',policy=B,target_y=-.15),
    dict(name='near_center_p05',policy=B,target_y=.05),dict(name='near_center_m05',policy=B,target_y=-.05),
    dict(name='restart',policy=B,restart=True),dict(name='pillars',policy=B,scene=dict(kind='pillars')),
    dict(name='near_x1.0',policy=B,scene=dict(kind='fixed',x=1.),target_x=1.),
    dict(name='far_x2.0',policy=B,scene=dict(kind='fixed',x=2.),target_x=2.),
    dict(name='ideal_center',policy=B,sensor='ideal')]
if __name__=='__main__':
    main('batch11',cases)

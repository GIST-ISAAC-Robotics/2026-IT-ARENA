"""along 지평 수정 뒤 먼 표적과 대표 대조만 다시 본다. 전수 반복 아님."""
from matrix import main

B=dict(surface=True,plan=True,onset=True)
cases=[dict(name='center_s17',policy=B),dict(name='center_s23',policy=B,seed=23),
    dict(name='edge_right',policy=B,target_y=-.15),dict(name='edge_left',policy=B,target_y=.15),
    dict(name='restart',policy=B,restart=True),
    dict(name='far_x2.0',policy=B,scene=dict(kind='fixed',x=2.),target_x=2.),
    dict(name='far_x1.5',policy=B,scene=dict(kind='fixed',x=1.5),target_x=1.5),
    dict(name='second_in_corridor',policy=B,scene=dict(kind='second',x2=1.9,y2=-.27))]
if __name__=='__main__':
    main('batch6',cases)

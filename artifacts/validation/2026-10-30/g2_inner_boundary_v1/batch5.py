"""후보 B 고정 뒤 처음 쓰는 시드, 대조군, 제거 비교, 민감도, 반례.

사전 등록: 후보 코드는 batch4 그대로다. 이 묶음 결과로 수치를 고치지 않는다.
시작 가속 3 m/s^2가 등록값이고 2, 5 m/s^2는 민감도다. 20초 일정, 복합 오차.
"""
from matrix import main

B=dict(surface=True,plan=True,onset=True)
cases=[dict(name=f'holdout_s{s}',policy=B,seed=s) for s in range(23,31)]
# 제거 비교: 요소 하나만 끈다.
cases+=[dict(name=f'no_surface_s{s}',policy=dict(B,surface=False),seed=s) for s in (17,18,19)]
cases+=[dict(name=f'no_plan_s{s}',policy=dict(B,plan=False),seed=s) for s in (17,18,19)]
# 시작 가속 민감도
cases+=[dict(name=f'a2_s{s}',policy=dict(B,onset_acceleration_mps2=2.),seed=s) for s in (17,18)]
cases+=[dict(name=f'a5_s{s}',policy=dict(B,onset_acceleration_mps2=5.),seed=s) for s in (17,18)]
# 기존 기능 대조: 기존/후보를 같은 시드로 비교
for label,policy in (('existing',None),('B',B)):
    cases+=[dict(name=f'{label}_edge_right',policy=policy,target_y=-.15),
            dict(name=f'{label}_edge_left',policy=policy,target_y=.15),
            dict(name=f'{label}_near_center_p05',policy=policy,target_y=.05),
            dict(name=f'{label}_near_center_m05',policy=policy,target_y=-.05),
            dict(name=f'{label}_restart',policy=policy,restart=True)]
# 반례와 장면 변형
for label,scene in (
        ('cutin_t2.0_u0.5',dict(kind='cutin',start=2.0,speed=.5,distance=.2,direction=-1.)),
        ('cutin_t3.5_u0.5',dict(kind='cutin',start=3.5,speed=.5,distance=.2,direction=-1.)),
        ('cutin_t4.3_u0.5',dict(kind='cutin',start=4.3,speed=.5,distance=.2,direction=-1.)),
        ('cutin_t3.5_u0.15',dict(kind='cutin',start=3.5,speed=.15,distance=.2,direction=-1.)),
        ('cutin_t4.3_u0.15',dict(kind='cutin',start=4.3,speed=.15,distance=.2,direction=-1.)),
        ('second_in_corridor',dict(kind='second',x2=1.9,y2=-.27)),
        ('right_side_occupied',dict(kind='second',x2=1.2,y2=-.30)),
        ('pillars',dict(kind='pillars')),
        ('yawed_0.3',dict(kind='fixed',yaw=.3)),
        ('near_x1.0',dict(kind='fixed',x=1.0)),
        ('far_x2.0',dict(kind='fixed',x=2.0))):
    cases.append(dict(name=f'B_{label}',policy=B,scene=scene,target_x=scene.get('x',1.2)))
if __name__=='__main__':
    main('batch5',cases)

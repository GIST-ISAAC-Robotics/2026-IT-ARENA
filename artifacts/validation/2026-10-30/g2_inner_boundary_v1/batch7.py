"""면 구간 제한 뒤 관측 조건 항, 정렬 상한, 왼쪽 강제, 먼 표적 비교.

다음 묶음의 등록값을 정하기 위한 비교이며 결과를 처음 쓰는 시드 검증으로 쓰지 않는다.
"""
from matrix import main

R=dict(surface=True,plan=True,onset=True)
D=dict(R,surface_error_budget_m=.06)
cases=[dict(name=f'term075_s{s}',policy=R,seed=s) for s in (17,18,19,23,24)]
cases+=[dict(name=f'term225_s{s}',policy=D,seed=s) for s in (17,18,19,23,24)]
cases+=[dict(name=f'snap20_s{s}',policy=dict(D,snap_budget_m=.02),seed=s) for s in (17,18,19)]
cases+=[dict(name=f'snap15_s{s}',policy=dict(D,snap_budget_m=.015),seed=s) for s in (17,18,19)]
cases+=[dict(name=f'left_term075_s{s}',policy=dict(R,force_side='left'),seed=s) for s in (17,18,19)]
cases+=[dict(name=f'left_term225_s{s}',policy=dict(D,force_side='left'),seed=s) for s in (17,18,19)]
for label,policy in (('term075',R),('term225',D)):
    cases+=[dict(name=f'{label}_far2.0',policy=policy,scene=dict(kind='fixed',x=2.),target_x=2.),
            dict(name=f'{label}_far1.5',policy=policy,scene=dict(kind='fixed',x=1.5),target_x=1.5),
            dict(name=f'{label}_second',policy=policy,scene=dict(kind='second',x2=1.9,y2=-.27)),
            dict(name=f'{label}_right_occupied',policy=policy,scene=dict(kind='second',x2=1.2,y2=-.30)),
            dict(name=f'{label}_edge_right',policy=policy,target_y=-.15),
            dict(name=f'{label}_edge_left',policy=policy,target_y=.15)]
if __name__=='__main__':
    main('batch7',cases)

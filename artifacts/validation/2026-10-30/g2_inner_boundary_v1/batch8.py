"""등록 후보(면 구간 제한, 관측 조건 항 .075, 정렬 상한 30 mm)의 반례 묶음.

사전 등록: 이 결과로 수치를 고치지 않는다. 편향 사례는 ±30 mm 상한을 넘지 않도록
상관 편향과 광선 잡음의 합을 30 mm로 맞춘다. 접촉·정지·정체를 그대로 기록한다.
"""
from matrix import main

B=dict(surface=True,plan=True,onset=True)
R=dict(B,surface=False)
cases=[]
# 실제 작은 돌출: 정렬이 가릴 수 있는 크기와 가릴 수 없는 크기
for label,scene in (('bump20_l06',dict(kind='bump',delta=.02,length=.06)),
        ('bump30_l06',dict(kind='bump',delta=.03,length=.06)),
        ('bump50_l06',dict(kind='bump',delta=.05,length=.06)),
        ('bump30_front',dict(kind='bump',delta=.03,length=.10,dx=.05)),
        ('bump30_thin',dict(kind='bump',delta=.03,length=.012)),
        ('wallbump20',dict(kind='wallbump',delta=.02,length=.06)),
        ('wallbump30',dict(kind='wallbump',delta=.03,length=.06)),
        ('wallbump50',dict(kind='wallbump',delta=.05,length=.06))):
    cases.append(dict(name=label,policy=B,scene=scene))
# 표면별 상관 편향 + 광선 잡음(합 ±30 mm)
for label,bias,width in (('bias_all_far20',dict(kind='all',value=.02),.01),('bias_all_near20',dict(kind='all',value=-.02),.01),
        ('bias_wall_far20',dict(kind='wall',value=.02),.01),('bias_target_far20',dict(kind='target',value=.02),.01),
        ('bias_all_far30',dict(kind='all',value=.03),0.),('bias_target_far30',dict(kind='target',value=.03),0.),
        ('bias_wall_far30',dict(kind='wall',value=.03),0.),('bias_view30',dict(kind='target_view',value=.03),0.)):
    cases.append(dict(name=label,policy=B,bias=bias,noise_half_width=width))
# 면 선택 변형: 기울어 끝 모서리만 가까운 면
cases+=[dict(name=f'yawed_{y}',policy=B,scene=dict(kind='fixed',yaw=y)) for y in (.05,.10,.15)]
# 움직이는 두 번째 물체: 정지 경계와 같은 시야에서 공존
for label,scene in (('second_lateral_far',dict(kind='second_moving',x2=1.9,y2=.12,speed=.5,distance=.39,direction=-1.,start=3.5)),
        ('second_lateral_slow',dict(kind='second_moving',x2=1.9,y2=.12,speed=.15,distance=.39,direction=-1.,start=2.0)),
        ('second_behind_boundary',dict(kind='second_moving',x2=1.65,y2=-.03,speed=.3,distance=.22,direction=-1.,start=3.0)),
        ('second_same_band_close',dict(kind='second_moving',x2=1.42,y2=0.,speed=.3,distance=.22,direction=-1.,start=4.0))):
    cases.append(dict(name=label,policy=B,scene=scene))
if __name__=='__main__':
    main('batch8',cases)

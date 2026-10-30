"""돌출부 옆을 실제로 지나도록 오른쪽 통로를 강제한 시험과 최종 후보의 처음 쓰는 시드."""
from matrix import main

B=dict(surface=True,plan=True,onset=True)
F=dict(B,force_side='right')
cases=[dict(name=f'final_s{s}',policy=B,seed=s) for s in range(31,39)]
for label,scene in (('bump20_l06',dict(kind='bump',delta=.02,length=.06)),('bump30_l06',dict(kind='bump',delta=.03,length=.06)),
        ('bump30_front',dict(kind='bump',delta=.03,length=.10,dx=.05)),('bump30_thin',dict(kind='bump',delta=.03,length=.012)),
        ('bump50_l06',dict(kind='bump',delta=.05,length=.06))):
    cases.append(dict(name=f'force_right_{label}',policy=F,scene=scene))
    cases.append(dict(name=f'nosnap_right_{label}',policy=dict(F,surface=False),scene=scene))
cases.append(dict(name='plan_tie_bump20',policy=B,scene=dict(kind='bump',delta=.02,length=.06)))
if __name__=='__main__':
    main('batch9',cases)

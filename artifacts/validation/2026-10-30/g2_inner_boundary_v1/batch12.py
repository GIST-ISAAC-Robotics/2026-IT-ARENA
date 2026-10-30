"""가장자리 대조의 복귀 뒤 정체가 후보 효과인지 시드로 나눈다. 기존/후보 같은 시드."""
from matrix import main

B=dict(surface=True,plan=True,onset=True)
cases=[]
for label,policy in (('existing',None),('B',B)):
    for seed in (18,19,20):
        cases.append(dict(name=f'{label}_edge_right_s{seed}',policy=policy,seed=seed,target_y=-.15))
if __name__=='__main__':
    main('batch12',cases)

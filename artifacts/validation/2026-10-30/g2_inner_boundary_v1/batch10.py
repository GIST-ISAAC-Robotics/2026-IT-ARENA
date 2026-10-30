"""후보가 실제로 옆을 지나는 더 작은 돌출. 등록 수치는 바꾸지 않는다."""
from matrix import main

B=dict(surface=True,plan=True,onset=True)
cases=[]
for delta in (.005,.01,.015):
    for seed in (17,18):
        cases.append(dict(name=f'bump{int(delta*1000)}_s{seed}',policy=B,seed=seed,scene=dict(kind='bump',delta=delta,length=.06)))
if __name__=='__main__':
    main('batch10',cases)

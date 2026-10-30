"""정렬 상한 비교를 등록 관측 조건 항 0.075 m/s에서 다시 측정한다. batch7의 snap 사례는 0.225 조건이었다."""
from matrix import main

B=dict(surface=True,plan=True,onset=True)
cases=[dict(name=f'snap{int(v*1000)}_s{s}',policy=dict(B,snap_budget_m=v),seed=s) for v in (.02,.015) for s in (17,18,19)]
if __name__=='__main__':
    main('batch13',cases)

"""closed1 작업자 재사용으로 패치가 누적되어 v=0으로 오염된 const_100_early 재측정."""
import multiprocessing as mp
from pathlib import Path

import counterfactual as c

CASES=[dict(name='const_100_early',mode='constant',p=.100,early=True),
       dict(name='const_085_early',mode='constant',p=.085,early=True)]


def run(case):
    c.BASE=Path(__file__).resolve().parent/'rerun2'
    return c.run(case)


if __name__=='__main__':
    with mp.get_context('spawn').Pool(2,maxtasksperchild=1) as pool:
        for s in pool.imap_unordered(run,CASES):
            print(s['case']['name'],s['final_x'],s['final_y'],s['last_phase'],s['passed_t'],s['first_bypass'],flush=True)

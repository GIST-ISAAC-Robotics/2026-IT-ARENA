"""Four-post tolerance envelope and static stiffness screen (mm, N, degrees, MPa).

Run: python scripts/screen_c1_mount_structure.py. No strength rating or CAD fit certification.
"""
import csv
import itertools
import math
from argparse import ArgumentParser
from pathlib import Path
from study_c1_mount import OUT, Layout, four, mask, overlap


def main():
    p = ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, default=OUT)
    dest=p.parse_args().out
    dest.mkdir(exist_ok=True,parents=True)
    jobs=dict(A=(0,80),B=(-30,80),C=(30,80),D=(0,35),E=(40,15),E2=(45,10),L=(60,80))
    with (dest/'tolerance.csv').open('w',encoding='utf-8',newline='') as f:
        w=csv.writer(f)
        w.writerow(['layout','effective_d_mm','nominal_total_deg','nominal_front100_deg','sampled_min_total_deg','sampled_max_total_deg','sampled_max_front100_deg','min_front_edge_deg'])
        for name,(s0,f0) in jobs.items():
            for d in (4,6,7,8,10):
                nominal=mask(Layout('','',s0,four(f0,-80,62,d)))
                values=[]
                for ds,df,dr,dyf,dyr in itertools.product((-5,0,5),repeat=5):
                    s,x=s0+ds,f0+df
                    posts=tuple((x,t*(62+dyf),d) for t in (-1,1))+tuple((-80+dr,t*(62+dyr),d) for t in (-1,1))
                    arcs=mask(Layout('','',s,posts))
                    values.append((sum(b-a for a,b in arcs),overlap(arcs,[(-100,100)]),
                        math.degrees(math.atan2(62+dyf,x-s)-math.asin((d/2)/math.hypot(x-s,62+dyf)))))
                numbers=[sum(b-a for a,b in nominal),overlap(nominal,[(-100,100)]),
                         min(v[0] for v in values),max(v[0] for v in values),
                         max(v[1] for v in values), min(v[2] for v in values)]
                w.writerow([name,d,*[round(v,3) for v in numbers]])
    # Tip load on an ideal clamped unperforated strip. Not a bound on real assembly flex.
    with (dest/'plate_screen.csv').open('w',encoding='utf-8',newline='') as f:
        w=csv.writer(f)
        w.writerow(['layout','front_overhang_mm','effective_strip_width_mm','thickness_mm','tip_load_N','E_MPa','tip_delta_mm','root_bending_stress_MPa'])
        force=.25*3*9.81
        for name,length in [('A/B/C/L',20),('D',65),('E',85),('E2-full',90),('E2-short',25),('F',180)]:
            for b in (40,60,80):
                for t in (1.5,2.0,2.5,3.0):
                    inertia=b*t**3/12
                    w.writerow([name,length,b,t,round(force,5),69000,
                        round(force*length**3/(3*69000*inertia),5), round(6*force*length/(b*t*t),4)])
    print('35 tolerance/diameter combinations (243 perturbations each); 72 plate cases.')


if __name__=='__main__':
    main()

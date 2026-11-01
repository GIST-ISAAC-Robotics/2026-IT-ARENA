import csv,sys
name=sys.argv[1];a=float(sys.argv[2]);b=float(sys.argv[3]);step=int(sys.argv[4]) if len(sys.argv)>4 else 5
import os
r=list(csv.DictReader(open(os.path.expanduser(f'~/arena_scratch/fc1/{name}.csv'),encoding='utf-8')))
def f(x):
    try:return f'{float(x):.3f}'
    except:return (x or '-')
print('t meas body out g1 | bind | fes_reason cap fixed axle | prep rate n disp sel ahead swept')
for i,x in enumerate(r):
    t=float(x['t'])
    if t<a or t>b: continue
    if i%step and x['bind'] not in('prepare_hold',) and not x['pers_n']=='1': continue
    print(f(x['t']),f(x['measured']),f(x['body']),f(x['out']),f(x['g1']),'|',x['bind'][:28],'|',(x['fes_reason'] or '-')[:20],f(x['fes_cap']),f(x['fixed_cap']),f(x['axle_to_patch']),'|',(x['prep_reason'] or '-')[:18],f(x['rate']),x['pers_n'],f(x['pers_disp']),x['pers_reason'][:14],f(x['ahead']),f(x['swept']))

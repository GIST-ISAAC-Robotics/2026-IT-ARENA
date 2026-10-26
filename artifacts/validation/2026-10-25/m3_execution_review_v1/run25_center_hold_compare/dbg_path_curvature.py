import json, sys, numpy as np
ev=[json.loads(l) for l in open(sys.argv[1],encoding='utf8')]
for e in ev:
    if e['kind']!='call': continue
    t=e['data']['input']['reference_s']
    if not any(abs(t-x)<1e-6 for x in (13.2, 16.9, 17.9, 18.42, 18.92)): continue
    p=np.array(e['data']['output'][2]['path_points'])
    d=np.diff(p,axis=0); h=np.unwrap(np.arctan2(d[:,1],d[:,0])); ds=np.hypot(d[:,0],d[:,1])
    k=np.diff(h)/((ds[1:]+ds[:-1])/2)
    print(round(t,2),'n',len(p),'first',p[0].round(3),'last',p[-1].round(3),'len',ds.sum().round(3),'max|k|',np.abs(k).max().round(3),'at s',np.cumsum(ds)[np.argmax(np.abs(k))].round(3), 'head deg', np.degrees(h[[0,len(h)//2,-1]]).round(2))

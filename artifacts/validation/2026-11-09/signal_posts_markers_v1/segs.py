import json,sys
rows=[json.loads(l) for l in open(sys.argv[1],encoding='utf-8')]
segs=[];cur=None
for r in rows:
    z = r['speed']<=1e-6 and r['base_speed']>1e-6
    d = abs(r['speed']-r['base_speed'])>0.05 or abs(r['steer']-r['base_steer'])>0.02
    tag = 'Z' if z else ('D' if d else None)
    if tag:
        if cur and cur['tag']==tag and r['t']-cur['t1']<0.05:
            cur['t1']=r['t']; cur['n']+=1; cur['reasons'].add(r['reason']); cur['near']=min(cur['near'], r['nearest_post_m'] or 99); cur['base']=max(cur['base'],r['base_speed']); cur['minv']=min(cur['minv'],r['speed']); cur['minb']=min(cur['minb'],r['base_speed'])
        else:
            cur=dict(tag=tag,t0=r['t'],t1=r['t'],n=1,reasons={r['reason']},near=r['nearest_post_m'] or 99,base=r['base_speed'],minv=r['speed'],minb=r['base_speed']); segs.append(cur)
for s in segs:
    print(s['tag'], round(s['t0'],2), round(s['t1'],2), s['n'], sorted(s['reasons']), 'near', round(s['near'],3), 'base', round(s['minb'],2),'-',round(s['base'],2), 'min_v', round(s['minv'],2))

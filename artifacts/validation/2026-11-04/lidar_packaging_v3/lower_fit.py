"""Coarse packaging search, not chassis/drive design. All boxes in mm. numpy required."""
import itertools,json,math
import numpy as np
from pathlib import Path
P=Path(__file__).resolve().parent
A=json.loads((P.parent/'lidar_mount_cad_reference'/'assembly.json').read_text(encoding='utf8'))
parts={r['label'][:2]:r['bounds']['size'] for r in A['top_level']}
GAP=3
def clear(a,b):
    return np.any((a[:,:3]>=b[3:]+GAP-1e-8)|(a[:,3:]<=b[:3]-GAP+1e-8),axis=1)
def candidates(dims,zlo,xrange):
    candidates=[]
    for (L,W,H) in dims:
        for x in np.arange(-97,97-L+0.01,3):
            if not xrange[0]<=x+L/2<=xrange[1]: continue
            for y in np.arange(-72,72-W+0.01,3):
                candidates.append([x,y,zlo,x+L,y+W,zlo+H])
    return np.array(candidates)
def steering_ok(arr,angles):
    # Four SAT axes; expand the wheel by 3 mm per side along its own axes.
    # Conservative plan projection (all tested part heights overlap the wheel height).
    keep=np.ones(len(arr),dtype=bool);cx=(arr[:,0]+arr[:,3])/2;cy=(arr[:,1]+arr[:,4])/2
    ex=(arr[:,3]-arr[:,0])/2;ey=(arr[:,4]-arr[:,1])/2
    for theta,side in itertools.product(angles,[-1,1]):
        a,b=math.cos(math.radians(theta)),math.sin(math.radians(theta)); A,B=abs(a),abs(b)
        x,y=cx-66.5,cy-side*60.95; L,W=32.2+GAP,13.05+GAP
        keep &= ((abs(x)>=ex+L*A+W*B) | (abs(y)>=ey+L*B+W*A) |
                 (abs(x*a+y*b)>=L+ex*A+ey*B) | (abs(-x*b+y*a)>=W+ex*B+ey*A))
    return keep
def solve(s,h,base,angles=(),diff=True,support=None,keep_motor_orientation=False):
    # CAD wheels zero steer; entire wheel AABBs (conservative in corners).
    static=[[x,y,0,x+64.4,y+26.1,64.4] for x in [-98.7,34.3] for y in [-74,47.9]]
    static+=[[s-27.8,-27.8,h-29.8-2,s+27.8,27.8,h+11.5]] # 2 mm support under C1
    if diff: static.append([-82,-16,18,-51,16,46]) # unmeasured trial diff reserve
    if support:
        front,ry=(10,52) if support=='JR' else (80,62)
        for x,offset in [(front,62),(-80,ry)]:
            for y in [-offset,offset]:
                static.append([x-3.5,y-3.5,0,x+3.5,y+3.5,130])
    dim_m=(parts['02'][1],parts['02'][0],parts['02'][2]);dim_b=(105,34,52);dim_s=(parts['10'][1],parts['10'][0],parts['10'][2])
    arrays=[]
    motor_orient=[dim_m] if keep_motor_orientation else [dim_m,(dim_m[1],dim_m[0],dim_m[2])]
    for ki,(k,dims,xr,preferred) in enumerate([('motor',motor_orient,(-100,45),(-13,22)),
            ('battery',[(105,34,52),(34,105,52),(105,52,34),(52,105,34)],(-100,65),(-15,-22)),
            ('servo',list(itertools.permutations(dim_s)),(36,93),(70,0))]):
        arr=candidates(dims,base[ki] if isinstance(base,tuple) else base,xr)
        for obstacle in static: arr=arr[clear(arr,np.array(obstacle))]
        if angles: arr=arr[steering_ok(arr,angles)]
        # Keep entire AABBs below the optical layer with a nominal 7 mm margin.
        arr=arr[arr[:,5]<=h-7]
        xy=(arr[:,:2]+arr[:,3:5])/2
        cost=np.sum((xy-np.array(preferred))**2,axis=1)
        if k=='servo': cost=cost+2200*(np.abs((arr[:,5]-arr[:,2])-dim_s[2])>0.1)
        arr=arr[np.argsort(cost,kind='stable')]
        arrays.append(arr)
    motor,battery,servo=arrays
    for mo in motor:
        ss=servo[clear(servo,mo)]
        if not len(ss): continue
        for ba in battery[clear(battery,mo)]:
            valid=ss[clear(ss,ba)]
            if len(valid):
                return {k:np.round(v,3).tolist() for k,v in zip(['motor','battery','servo'],[mo,ba,valid[0]])}
    return None
if __name__=='__main__':
    out={}
    for s,h,z in [(50,75,15),(50,85,15),(50,90,15),(50,95,15),(50,95,10),(50,97,10),(50,100,12),(50,100,15),(50,102,15),(65,75,15),(65,95,15),(65,102,15)]:
        out[f'x{s}_h{h}_floor{z}']=solve(s,h,z)
        print(s,h,z,out[f'x{s}_h{h}_floor{z}'],flush=True)
    for s,h,z,diff in [(50,75,15,True),(50,102,15,True),(65,102,15,True),(50,102,15,False)]:
        out[f'x{s}_h{h}_floor{z}_steer26_diff{int(diff)}']=solve(s,h,z,list(range(-26,27,2)),diff)
    proposal=out['x50_h102_floor15']
    out['straight_proposal_steering_interference_with_inflated_box']={
        k:[t for t in range(-26,27,2) if not steering_ok(np.array([v]),[t])[0]] for k,v in proposal.items()}
    for support,s in [('JR',50),('C',50),('C',65)]:
        for h in [75,85,90,95,100,102,105,110]:
            best=None
            # Same full reserve, sampled steering; heavy pack and motor independently stepped.
            for zm,zb in sorted(itertools.product([12,15,30,49,55],[12,15,30,49,55,60,63]),key=lambda pair:sum(pair)):
                trial=solve(s,h,(zm,zb,15),list(range(-26,27,2)),True,support)
                if trial:
                    best=trial;break
            out[f'{support}{s}_h{h}_stepped_steer26_diff1_posts']=best
            print('stepped',support,s,h,best,flush=True)
    for h in [95,102,105,110]:
        best=None
        for zm,zb in sorted(itertools.product([12,15,30,49,55],[12,15,30,49,55,60,63]),key=sum):
            best=solve(50,h,(zm,zb,15),list(range(-26,27,2)),True,'JR',True)
            if best:break
        out[f'JR50_h{h}_stepped_longitudinal_motor']=best
    (P/'lower_fit.json').write_text(json.dumps(out,indent=2)+'\n')

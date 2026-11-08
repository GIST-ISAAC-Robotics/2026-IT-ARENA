"""동일 물리시각의 접촉쌍/구동 운동 대조. 원본 JSONL과 제어 입력은 수정하지 않는다."""
from collections import Counter,defaultdict
from pathlib import Path
import bisect,hashlib,json,math,statistics
HERE=Path(__file__).resolve().parent;run=HERE/'run1';read=lambda n:json.loads((run/n).read_bytes())
trace=read('trace.json');drive=[json.loads(json.loads(l)['data']) for l in (run/'drivetrain_gz.jsonl').read_bytes().splitlines()];assert all(a['sim_time_s']<b['sim_time_s'] for a,b in zip(drive,drive[1:]));times=[r['sim_time_s'] for r in drive]
# 33–35m 동안 높은 차체속도에서 어느 한쪽이 거의 잠긴 구간을 먼저 찾는다. 힘을 보고 구간을 고르지 않는다.
a=next(r['dynamics']['sim_time_s'] for r in trace if r['progress_m']>=33.)
b=next(r['dynamics']['sim_time_s'] for r in trace if r['progress_m']>=35.)
windows=defaultdict(set);motion=defaultdict(list)
for d in drive:
    t=d['sim_time_s'];v=d['truth_longitudinal_mps'];wl,wr=[d[k]*.0325 for k in ('left_speed_rad_s','right_speed_rad_s')]
    if not a<=t<=b:continue
    group='right_slow' if v>1.5 and wr<.5 and abs(wl-v)<.25 else 'left_slow' if v>1.5 and wl<.5 and abs(wr-v)<.25 else 'both_rolling' if v>1.5 and max(abs(wl-v),abs(wr-v))<.25 else 'other'
    windows[group].add(round(t*1e9));motion[group].append(dict(t=t,body=v,left=wl,right=wr,motor=d['motor_torque_nm'],target=d['applied_target_mps'],steer_left=d['left_steering_rad'],steer_right=d['right_steering_rad'],yaw=d['truth_yaw_rad']))
keys={k:g for g,t in windows.items() for k in t};rows=defaultdict(dict);flips=Counter();labels=Counter();skipped=Counter()
with (run/'wheel_contacts.jsonl').open('rb') as f:
    for line in f:
        row=json.loads(line);sec=row['header']['stamp'];t=int(sec.get('sec',0))*10**9+int(sec.get('nsec',0));group=keys.get(t)
        if group is None:continue
        for c in row.get('contact',[]):
            names=[c[f'collision{i}']['name'] for i in (1,2)]
            selected=[i for i,n in enumerate(names) if '::rear_' in n and n.startswith('arena_car::')]
            if len(selected)!=1:continue
            idx=selected[0];wheel=names[idx].split('::')[1];other=names[1-idx]
            vector=[0.,0.,0.];valid=True
            for j,w in enumerate(c.get('wrench',[])):
                one=w.get('body1Wrench',{}).get('force');two=w.get('body2Wrench',{}).get('force');n=c.get('normal',[])
                if one is None or two is None or j>=len(n) or abs(n[j].get('z',0))<.99:valid=False;break
                f1=[one.get(k,0.) for k in 'xyz'];f2=[two.get(k,0.) for k in 'xyz']
                if max(abs(x+y) for x,y in zip(f1,f2))>1e-5:valid=False;break
                wheel_by_label=(f1,f2)[idx]
                if wheel_by_label[2]<-.001:flips[group,wheel]+=1
                # 지면의 수직항은 위쪽인 작용쌍의 크기로 비교. 충돌명만으로 차체에 작용한 부호를 단정하지 않는다.
                fup=f1 if f1[2]>=0 else f2
                for axis in range(3):vector[axis]+=fup[axis]
            if not valid:skipped[group,wheel]+=1;continue
            labels[group,wheel,other]+=1
            bytime=rows[group,wheel].setdefault(t,{})
            if other in bytime:assert all(abs(x-y)<1e-7 for x,y in zip(bytime[other],vector))
            bytime[other]=vector
result={}
for g,ms in motion.items():
    summary=dict(drive_samples=len(ms),from_s=ms[0]['t'],to_s=ms[-1]['t'],motion_median={k:statistics.median(r[k] for r in ms) for k in ms[0] if k!='t'},wheels={})
    for wheel in ('rear_left_wheel','rear_right_wheel'):
        vals=[];long=[];wdata=rows[g,wheel]
        for m in ms:
            surfaces=wdata.get(round(m['t']*1e9))
            if not surfaces:continue
            x,y,z=[sum(v[i] for v in surfaces.values()) for i in range(3)];vals.append(z);long.append(x*math.cos(m['yaw'])+y*math.sin(m['yaw']))
        summary['wheels'][wheel]=dict(exact_stamps=len(vals),missing_stamps=len(ms)-len(vals),median_upward_pair_z_n=statistics.median(vals) if vals else None,median_upward_pair_body_long_n=statistics.median(long) if long else None,label_selected_downward_points=flips[g,wheel],rejected_contacts=skipped[g,wheel],other_labels={k[2]:v for k,v in labels.items() if k[:2]==(g,wheel)})
    result[g]=summary
out=dict(scope='exact common telemetry timestamps only, no zero-fill and no impulse claim. Upward member of paired wrench on near-horizontal contact; not signed wheel force solely from collision labels.',progress_window_m=[33,35],time_window_s=[a,b],groups=result,peak_body_mps=max(r['truth_longitudinal_mps'] for r in drive),input_hashes=json.loads((HERE/'reader_summary.json').read_bytes()))
(HERE/'force_review.json').write_text(json.dumps(out,indent=1)+'\n');print(json.dumps({g:dict(n=q['drive_samples'],time=[q['from_s'],q['to_s']],wheels={w:[v['exact_stamps'],v['median_upward_pair_z_n'],v['median_upward_pair_body_long_n']] for w,v in q['wheels'].items()}) for g,q in result.items()},indent=1))

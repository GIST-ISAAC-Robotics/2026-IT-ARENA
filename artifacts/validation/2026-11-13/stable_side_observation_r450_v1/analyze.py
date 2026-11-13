"""r441 원 tracker와 제한 후보 진단. 전체 plan/주행/공통 변경 없음."""
import collections
from dataclasses import replace
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import zipfile

B=Path(__file__).resolve().parent; R441=B.parent/'crop_crawl_drive_r441_v1'; O=B/'result_v1'
O.mkdir(exist_ok=False)
sha=lambda b:hashlib.sha256(b).hexdigest()
with zipfile.ZipFile(R441/'sources.zip') as z:
    original=z.read('src/arena_autonomy/arena_autonomy/static_side_return.py')
    (B/'original_static_side_return.py').write_bytes(original)
    source_refs={}
    for name,ranges in [('pass_release_trial.py',[(318,331),(558,650)]),
                        ('local_follow.py',[(458,464),(580,605)]),('bypass_handoff.py',[(608,615)])]:
        raw=z.read('src/arena_autonomy/arena_autonomy/'+name);source_refs[name]=sha(raw)
        excerpt='\n'.join(f'{i}: {line}' for i,line in enumerate(raw.decode().splitlines(),1) if any(a<=i<=b for a,b in ranges))
        (B/(name+'.excerpt.txt')).write_text(excerpt+'\n',encoding='utf8')
spec=importlib.util.spec_from_file_location('frozen_static_side_r441',B/'original_static_side_return.py')
mod=importlib.util.module_from_spec(spec);sys.modules[spec.name]=mod;spec.loader.exec_module(mod)
Tracker,Config=mod.StaticSideTracker,mod.StaticSideConfig

class StableRelativeDiagnostic(Tracker):
    """원 reset/residual를 유지, 최소8행/.7초와 이동 문턱0만 별도 대조."""
    def __init__(self):
        super().__init__(replace(Config(),minimum_scans=8,minimum_span_s=.7,minimum_travel_m=0.))
        self.strict_float_result=False
    def observe(self,stamp,front_ds):
        self.strict_float_result=super().observe(stamp,front_ds)
        # 별도 진단: ROS 정수 ns로 표현 가능한 .7초의 float 경계만 구분.
        # strict 결과도 보존하며 원 helper는 바꾸지 않는다.
        return bool(self.rows and len(self.rows)>=8 and round((stamp-self.rows[0][0])*1e9)>=700_000_000)

def snapshot(t):
    r=t.rows
    return dict(rows=len(r),travel_m=t.travel_m,reference_s=t.reference_s,reference_speed=t.reference_speed,
        span_s=r[-1][0]-r[0][0] if r else None,window_travel_m=r[-1][2]-r[0][2] if r else None,
        residual_m=r[-1][1]-r[0][1]+r[-1][2]-r[0][2] if r else None,row_data=r.copy())

trace=[json.loads(l) for l in (R441/'run1/follow_decisions.jsonl').open(encoding='utf8')]
header=trace[0]['data'];assert header['follow_config']['static_side_release_trial'] is True
seen={};records=[]
for r in trace:
    if r['kind']!='call':continue
    d=r['data'];c=d['compute'];key=(c['generation'],c['sequence'],c['stage'])
    if key in seen:
        assert seen[key]['input']==d['input'] and seen[key]['output']==d['output'];continue
    seen[key]=d
    if d['input']['reference_s']<32.:continue
    n=d['output'][2].get('normal',{});e=n.get('selector_evidence');signed=d['input']['speed_mps']
    planning=abs(signed) if header['follow_config']['signed_motion_policy']=='forward_drive' and signed<0 else signed
    records.append(dict(seq=r['seq'],reference_s=d['input']['reference_s'],source_first_s=n.get('source_first_s'),
        signed_measured_mps=signed,tracker_speed_mps=planning,normal_reason=n.get('reason'),selector_evidence=e,
        recorded_sides=n.get('selector_sides'),recorded_mode=n.get('selector_mode'),can_advance=e is not None,
        can_observe=bool(e and e.get('fresh') and 'left' in e and 'right' in e)))
old={s:Tracker() for s in ('left','right')};new={s:StableRelativeDiagnostic() for s in ('left','right')};results=[]
for row in records:
    out=dict(row,sides={})
    for side in ('left','right'):
        a,b=old[side],new[side];prea,preb=snapshot(a),snapshot(b)
        if row['can_advance']:
            a.advance_to(row['reference_s'],row['tracker_speed_mps']);b.advance_to(row['reference_s'],row['tracker_speed_mps'])
        mida,midb=snapshot(a),snapshot(b)
        if row['can_observe']:
            e=row['selector_evidence'][side];front=e['frontmost_ds_m']
            aa=a.observe(row['reference_s'],front);bb=b.observe(row['reference_s'],front);expected=e.get('world_fixed_return',False)
        else:aa=bb=expected=front=None
        out['sides'][side]=dict(front_ds=front,original_result=aa,original_recorded=expected,original_boolean_matches=aa==expected,
            stable_relative=bb,stable_strict_float=b.strict_float_result if row['can_observe'] else None,
            before_advance_original=prea,after_advance_original=mida,after_observe_original=snapshot(a),
            before_advance_candidate=preb,after_advance_candidate=midb,after_observe_candidate=snapshot(b))
    results.append(out)

def synth(name,samples,description):
    a,b=Tracker(),StableRelativeDiagnostic();rows=[]
    for stamp,speed,front in samples:
        a.advance_to(stamp,speed);b.advance_to(stamp,speed)
        aa=a.observe(stamp,front);bb=b.observe(stamp,front)
        rows.append(dict(t=stamp,ego_speed=speed,front_ds=front,original=aa,stable_relative=bb,strict_float=b.strict_float_result,
                         original_state=snapshot(a),candidate_state=snapshot(b)))
    return dict(name=name,physical_description=description,rows=rows,original_accepted=sum(r['original'] for r in rows),
        candidate_accepted=sum(r['stable_relative'] for r in rows),first_candidate_s=next((r['t'] for r in rows if r['stable_relative']),None))

times=[i/10 for i in range(31)];synthetic=[]
for v in (.1,.15):
    synthetic.append(synth('stationary_return_ego_'+str(v),[(t,v,.6-v*t) for t in times],'물체 정지, 자차 일정 전진.'))
for v in (.1,.05):
    synthetic.append(synth('same_speed_neighbor_'+str(v),[(t,v,.6) for t in times],'옆차/자차 같은 속도 전진. 세계 고정 물체 아님.'))
for v in (.2,.05):
    synthetic.append(synth('stopped_neighbor_then_moves_'+str(v),[(t,0.,.6+v*max(0.,t-1.)) for t in times],
                           '자차 정지, 옆차가 1.0초 이후 이동 시작.'))
synthetic.append(synth('missing_return_flicker',[(t,0.,None if i in (6,12,18,24,30) else .6) for i,t in enumerate(times)],
                       'front_ds 자체 소실, observe(None)은 rows를 지운다.'))
synthetic.append(synth('finite_two_three_point_flicker',[(t,0.,.6+.002*(i%2)) for i,t in enumerate(times)],
                       '점수2/3 변동이나 front_ds 유한. helper는 present와 무관하게 observe.'))
gap_times=[i/10 for i in range(11)]+[1.5+i/10 for i in range(16)]
synthetic.append(synth('long_gap',[(t,0.,.6) for t in gap_times],'.5초 공백으로 advance_to 전체 reset.'))
synthetic.append(synth('signed_negative_helper_input',[(t,-.1 if t==1.1 else 0.,.6) for t in times],
                       'helper 직접 음수 입력은 reset. 실제 forward_drive abs와 별개.'))
stall=[r for r in results if r['seq']>=895]
summary=dict(scope='기록 evidence의 helper 호출만 재생. 전체 plan/selector/미래 상태 복원 아님.',
    source_sha256=sha(original),source_refs=source_refs,source_archive_sha256=sha((R441/'sources.zip').read_bytes()),
    original_config=vars(Config()),candidate_config=vars(new['left'].config),
    time_boundary='strict float와 별도 정수ns .7초 진단 동시 보존. 원 helper 불변.',
    retained_records=len(results),stall_records=len(stall),
    original_helper_boolean_mismatches=[dict(seq=r['seq'],side=s,actual=r['sides'][s]['original_result'],expected=r['sides'][s]['original_recorded'])
        for r in results for s in ('left','right') if not r['sides'][s]['original_boolean_matches']],
    original_world_fixed_events=[dict(seq=r['seq'],t=r['reference_s'],side=s) for r in results for s in ('left','right') if r['sides'][s]['original_result']],
    first_candidate={s:next((r['reference_s'] for r in results if r['sides'][s]['stable_relative']),None) for s in ('left','right')},
    stall_candidate_both=sum(all(r['sides'][s]['stable_relative'] for s in ('left','right')) for r in stall),
    stall_strict_float_both=sum(all(r['sides'][s]['stable_strict_float'] for s in ('left','right')) for r in stall),
    stall_point_counts={s:dict(collections.Counter(str(r['selector_evidence'][s]['points']) for r in stall)) for s in ('left','right')},
    stall_nonfinite_front={s:sum(r['sides'][s]['front_ds'] is None for r in stall) for s in ('left','right')},
    independent_scope_limit='누락 선행 plan을 채우지 않았다. helper boolean 일치는 전체 상태 exact 증거 아님.',
    original_signed_negative_events=[dict(seq=r['seq'],signed=r['signed_measured_mps'],tracker=r['tracker_speed_mps']) for r in results if r['signed_measured_mps']<0],
    synthetic=[{k:v for k,v in s.items() if k!='rows'} for s in synthetic],
    common_modified=False,ros_started=False,wsl_used=False,new_driving=False,point_occupancy_removed=False)
for name,obj in [('observed_helper_rows.json',results),('synthetic_rows.json',synthetic),('summary.json',summary)]:
    (O/name).write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(summary,ensure_ascii=False,indent=2))

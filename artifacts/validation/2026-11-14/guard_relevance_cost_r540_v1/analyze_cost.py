"""저장 r538 비용 구조와 네 대표 크기의 Windows fallback sweep만 분석한다."""
import ast
from collections import Counter
import difflib
import hashlib
import json
import math
from pathlib import Path
import platform
import time

import numpy as np

A=Path(__file__).resolve().parent
P=A.parent/'guard_memory_relevance_r538_v1'
FROZEN=P/'stage/src/arena_autonomy/arena_autonomy'
EXPECTED='dc81494c5162d0e07851e835bee7dfc96fac303a857fe0c922da50959647b26d'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def quantiles(values):
    v=sorted(values)
    def at(q):
        n=(len(v)-1)*q;i=int(n)
        return v[i]+(v[min(i+1,len(v)-1)]-v[i])*(n-i)
    return dict(n=len(v),median=at(.5),p95=at(.95),maximum=v[-1]) if v else dict(n=0)


def work(row,dedup=False):
    c=row['crawl'];h=c['horizon_m'];poses=np.arange(0.,h+.001,.04)
    repeat=1 if dedup and c['current_angle_rad']==c['previous_angle_rad'] else 2
    total=0
    for suffix,n in [('memory',row['active'] or 0),('raw',row['raw_points'])]:
        for angle in ('current','previous')[:repeat]:
            clear=c[angle+'_'+suffix+'_clearance_m']
            if clear<h:
                # zero clearance may mean hit index0 or1; both are in the same8-row batch.
                index=0 if clear==0 else int(round((clear+.04)/.04))
                evaluated=min(len(poses),8*(index//8+1))
            else:evaluated=len(poses)
            total+=n*evaluated
    return dict(calls=repeat*(1+bool(row['active'])),evaluated_point_pose_pairs=total,
                full_no_hit_point_pose_pairs=repeat*(row['raw_points']+(row['active'] or 0))*len(poses))


def sweep_section(source,swept):
    tree=ast.parse(source)
    fn=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='limited_memory_command')
    body=next(x for x in fn.body if isinstance(x,ast.Try)).body
    start=next(i for i,x in enumerate(body) if isinstance(x,ast.Assign) and isinstance(x.targets[0],ast.Name) and x.targets[0].id=='angles')
    selected=[]
    for x in body[start:]:
        if isinstance(x,ast.Assign) and isinstance(x.targets[0],ast.Name) and x.targets[0].id in ('memory_relevant','clearance'):
            break
        selected.append(x)
    base=ast.parse('def section(retained,current,steering,previous,wheelbase,horizon,length,width):\n return memory_clearances,raw_clearances\n')
    base.body[0].body=selected+base.body[0].body
    ns={'swept_limit':swept}
    exec(compile(ast.fix_missing_locations(base),'<frozen-sweep-section>','exec'),ns)
    return ns['section']


def main():
    assert sha(FROZEN/'guard_memory_command.py')==EXPECTED
    source=(FROZEN/'guard_memory_command.py').read_text(encoding='utf8')
    old=(P/'before.py').read_text(encoding='utf8')
    needle='''        angles=(steering,previous)
        memory_clearances=[swept_limit(retained,a,wheelbase,horizon=horizon,
            length=length,width=width) if len(retained) else horizon for a in angles]
        raw_clearances=[swept_limit(current,a,wheelbase,horizon=horizon,
            length=length,width=width) for a in angles]
'''
    replacement=needle.replace('angles=(steering,previous)',
        'angles=(steering,) if steering==previous else (steering,previous)')+'''        # 동일 호출의 정확히 같은 각도만 재사용한다. 기억/관측 갱신은 위에서 그대로 수행했다.
        if len(angles)==1:
            memory_clearances*=2
            raw_clearances*=2
'''
    assert source.count(needle)==1
    optimized=source.replace(needle,replacement,1)
    candidate=A/'stage/guard_memory_command.py';candidate.parent.mkdir(exist_ok=True)
    candidate.write_text(optimized,encoding='utf8')
    (A/'same_angle.patch').write_text(''.join(difflib.unified_diff(source.splitlines(True),optimized.splitlines(True),
        fromfile='a/src/arena_autonomy/arena_autonomy/guard_memory_command.py',
        tofile='b/src/arena_autonomy/arena_autonomy/guard_memory_command.py')),encoding='utf8')
    rows={key:json.loads((P/name).read_bytes()) for key,name in
          [('before','common_replay/rows.json'),('r538','candidate_replay/rows.json')]}
    metrics={}
    for label,values in rows.items():
        active=[r for r in values if r.get('crawl') and 'horizon_m' in r['crawl']]
        for dedup in ([False] if label=='before' else [False,True]):
            key=label+('_same_angle' if dedup else '')
            costs=[work(r,dedup) for r in active]
            metrics[key]=dict(helper_sweep_calls=len(active),
                exact_equal_angle_calls=sum(r['crawl']['current_angle_rad']==r['crawl']['previous_angle_rad'] for r in active),
                sweep_function_calls=sum(r['calls'] for r in costs),
                evaluated_point_pose_pairs=sum(r['evaluated_point_pose_pairs'] for r in costs),
                full_no_hit_point_pose_pairs=sum(r['full_no_hit_point_pose_pairs'] for r in costs),
                horizon_m=quantiles(r['crawl']['horizon_m'] for r in active),
                maximum_retained=max(r['active'] or 0 for r in active),
                maximum_retained_horizon_m=max(r['crawl']['horizon_m'] for r in active if r['active']==113),
                raw_point_range=[min(r['raw_points'] for r in active),max(r['raw_points'] for r in active)])
    local=(FROZEN/'local_path.py').read_text(encoding='utf8')
    fn=next(x for x in ast.parse(local).body if isinstance(x,ast.FunctionDef) and x.name=='swept_limit')
    env={'np':np,'math':math}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'<frozen-swept_limit>','exec'),env)
    swept=env['swept_limit']
    sections={label:sweep_section(text,swept) for label,text in [('before',old),('r538',source),('r538_same_angle',optimized)]}
    extracts=A.parent/'field_wall_resource_drive_r530_v1/guard_event_extract'
    specs=[('first_positive_guard',json.loads((extracts/'first_positive_guard.json').read_bytes())['raw']['data']['reference_s']),
           ('maximum_retained',7.82),
           ('first_memory_normal_cap_restored',json.loads((extracts/'first_memory_normal_cap_restored.json').read_bytes())['raw']['data']['reference_s']),
           ('last_requested_above_crawl_cap',json.loads((extracts/'last_requested_above_crawl_cap.json').read_bytes())['raw']['data']['reference_s'])]
    benchmarks=[]
    for name,at in specs:
        pair={label:next(r for r in data if abs(r['now']-at)<1e-8) for label,data in rows.items()}
        row=pair['r538'];d=row['crawl']
        # 실제 저장 크기/각도/horizon을 사용하되 좌표는 no-hit 상한용 합성이다.
        # 원 guard geometry/상태/전체callback 비용 재생으로 주장하지 않는다.
        retained=np.full((row['active'] or 0,2),5.);current=np.full((row['raw_points'],2),5.)
        samples={label:[] for label in sections}
        for repeat in range(9):
            for label in list(sections)[repeat%3:]+list(sections)[:repeat%3]:
                horizon=pair['before' if label=='before' else 'r538']['crawl']['horizon_m']
                began=time.perf_counter_ns()
                output=sections[label](retained,current,d['current_angle_rad'],d['previous_angle_rad'],.128,horizon,.207,.15)
                elapsed=(time.perf_counter_ns()-began)/1e6
                assert output==([horizon,horizon],[horizon,horizon])
                samples[label].append(elapsed)
        benchmarks.append(dict(case=name,reference_s=at,raw_points=row['raw_points'],retained=row['active'],
            current_angle=d['current_angle_rad'],previous_angle=d['previous_angle_rad'],
            before_horizon=pair['before']['crawl']['horizon_m'],candidate_horizon=d['horizon_m'],
            windows_sweep_section_ms={key:quantiles(values) for key,values in samples.items()}))
    result=dict(scope='Windows Python/NumPy fallback, scalar work accounting and bounded sweep-section fixture only; no ARM estimate',
        source_hashes=dict(before=sha(P/'before.py'),r538=EXPECTED,stage=sha(candidate),local_path=sha(FROZEN/'local_path.py')),
        input_hashes={str(f.relative_to(P)):sha(f) for f in [P/'common_replay/rows.json',P/'candidate_replay/rows.json']},
        extract_hashes={f.name:sha(f) for f in extracts.glob('*.json')},
        python=platform.python_version(),numpy=np.__version__,platform=platform.platform(),
        stored_accounting=metrics,windows_sweep_fixtures=benchmarks,
        limitation='Point-pose work is derived from saved horizons/clearances and8-row batching, not measured CPU cycles. Synthetic no-hit points avoid claiming exact saved geometry or whole-helper timing.')
    (A/'cost_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    assert sha(FROZEN/'guard_memory_command.py')==EXPECTED
    print(json.dumps(dict(stored_accounting=metrics,windows_sweep_fixtures=benchmarks),ensure_ascii=False))


if __name__=='__main__':main()

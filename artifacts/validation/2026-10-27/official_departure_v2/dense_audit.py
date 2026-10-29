"""동일20×15cm 차량 상자의 모든 쌍 간격. 평가 전용이며 제어기에 연결하지 않는다."""
from itertools import combinations
import json
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
import g1_traffic_eval as ev


def pair_metrics(streams, first, last, curvature=3.):
    results={}
    for (a,aa),(b,bb) in combinations(streams.items(),2):
        valid=ev._pose_rows_valid(aa) and ev._pose_rows_valid(bb)
        coverage=valid and all(ev.stream_coverage([r[0] for r in rows],first,last,ev.POSE_MAX_GAP_S)['valid']
                               for rows in (aa,bb))
        span=ev.window_tuples(aa,first,last) if valid else []
        sampled=ev.sampled_gaps(span,bb,.20,.15,ev.POSE_MAX_GAP_S) if coverage else []
        swept=ev.swept_gap_lower_bounds(span,bb,.20,.15,.01,curvature,ev.POSE_MAX_GAP_S) if coverage else []
        gaps=[r['gap_m'] for r in sampled if r['gap_m'] is not None]
        bounds=[r['lower_bound_m'] for r in swept if r['lower_bound_m'] is not None]
        complete=bool(sampled) and bool(swept) and len(gaps)==len(sampled) and len(bounds)==len(swept)
        overlaps=sum(r['overlap_area_m2'] is not None and r['overlap_area_m2']>1e-9 for r in sampled)
        passed=bool(coverage and complete and overlaps==0 and min(bounds)>0)
        results[a+'|'+b]=dict(passed=passed,coverage=bool(coverage),complete=complete,
            sampled_overlap_samples=overlaps,min_sampled_gap_m=min(gaps,default=None),
            min_swept_lower_bound_m=min(bounds,default=None),sample_count=len(sampled),interval_count=len(swept))
    return dict(passed=bool(results) and all(r['passed'] for r in results.values()),pairs=results,
                scope='equal20x15cm planar rectangles, dense samples and conservative swept bounds; not hardware margin')


def load_and_check(out, report):
    first=report.get('case_window',{}).get('green_sim_s')
    if first is None:
        return dict(passed=False,pairs={},reason='no_green_anchor')
    def rows(name):
        return [tuple(json.loads(line)) for line in (out/name).read_text(encoding='utf-8').splitlines()]
    streams={'arena_car':rows('ego_poses.jsonl'),'eval_g1b_actor':rows('actor_poses.jsonl')}
    raw=json.loads((out/'additional_actors.json').read_text(encoding='utf-8'))
    streams.update(raw['poses'])
    if not streams['arena_car']:
        return dict(passed=False,pairs={},reason='no_ego_poses')
    first-=1.
    last=streams['arena_car'][-1][0]
    return pair_metrics(streams,first,last)


def self_check():
    times=[i*.01 for i in range(101)]
    a=[(t,0.,0.,0.) for t in times]
    clear=[(t,0.,.18,0.) for t in times]
    overlap=[(t,0.,.14,0.) for t in times]
    cut=[r for r in clear if not .4<r[0]<.6]
    assert pair_metrics({'a':a,'b':clear},0.,1.)['passed']
    assert not pair_metrics({'a':a,'b':overlap},0.,1.)['passed']
    assert not pair_metrics({'a':a,'b':cut},0.,1.)['passed']
    assert not pair_metrics({'a':a,'b':[]},0.,1.)['passed']
    # 주 actor만 멀고 추가 차량끼리 겹친 경우도 검출한다.
    third=[(t,0.,.19,0.) for t in times]
    result=pair_metrics({'ego':a,'b':clear,'c':third},0.,1.)
    assert not result['passed'] and not result['pairs']['b|c']['passed']
    print('dense pair audit: five positive/negative controls passed')


if __name__=='__main__':self_check()

"""보존된 후보 함수들을 명시 선택 모듈로 옮긴다. 새 허용 조건을 만들지 않는다."""
import ast
import hashlib
import json
from pathlib import Path
import zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
CHANGED=['src/arena_autonomy/arena_autonomy/local_follow.py',
         'src/arena_autonomy/arena_autonomy/pass_release_trial.py']
with zipfile.ZipFile(A/'before_sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
    for name in CHANGED:archive.write(ROOT/name,name)


def function(relative,name):
    path=ROOT/'artifacts/validation'/relative
    raw=path.read_text(encoding='utf-8-sig')
    return ast.get_source_segment(raw,next(n for n in ast.parse(raw).body
        if isinstance(n,ast.FunctionDef) and n.name==name))


body=function('2026-11-11/body_nonreturn_r136_v1/candidate.py','same_path_with_body')
body=body.replace('same_path_with_body(', '_same_path_with_body(').replace('old=ORIGINAL(', 'old=ps._same_path_risk_original(')
adjacent=function('2026-11-12/adjacent_mount_gap_r149_v1/candidate.py','same_path_adjacent')
adjacent=adjacent.replace('same_path_adjacent(', 'same_path_risk(').replace('body.same_path_with_body(', '_same_path_with_body(')
recheck=function('2026-11-11/guard_recheck_r139_v1/candidate.py','guard_recheck')
recheck=recheck.replace('ORIGINAL(owner,', 'ps.PassReleaseSelector._guard_curve_original(owner,')
horizon=function('2026-11-12/guard_horizon_r148_v1/candidate.py','guard_horizon').replace('prior.guard_recheck(', 'guard_recheck(')
eligible_adjacent=function('2026-11-12/adjacent_low_retry_r178_v1/candidate.py','eligible').replace('def eligible(', 'def eligible_adjacent(')
guard_adjacent=function('2026-11-12/adjacent_low_retry_r178_v1/candidate.py','guard')
guard_adjacent=guard_adjacent.replace('def guard(', 'def guard_adjacent(').replace('_old_guard(', 'guard_horizon(').replace('if not eligible(', 'if not eligible_adjacent(')
eligible_approach=function('2026-11-12/guard_approach_cap_r181_v1/candidate.py','eligible').replace('def eligible(', 'def eligible_approach(')
guard=function('2026-11-12/guard_approach_cap_r181_v1/candidate.py','guard')
guard=guard.replace('def guard(', 'def guard_curve(').replace('_old_guard(', 'guard_adjacent(').replace('if not eligible(', 'if not eligible_approach(')
prefix='''"""명시 선택형 기존 GUARD 관측/저속 정책. 전역 patch나 artifact import 없음.

r136/r139/r148/r149/r178/r181의 검사 순서와 반환 근거를 보존한다.
기본 original_v1은 이 모듈을 호출하지 않는다. 현재 경로 재검사/평가 지연/
최근 참조 보존과 독립이며 미완주 후보의 기본 채택이나 실차 승인이 아니다.
"""
import math
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.current_path_recheck._gaps import bounded_body_nonreturn, bounded_adjacent_structure
from arena_autonomy.known_structure import parse_sectors

CAPS = (.25, .15)


'''
out=ROOT/'src/arena_autonomy/arena_autonomy/guard_observation_policy.py'
with out.open('x',encoding='utf-8') as stream:
    stream.write(prefix+'\n\n\n'.join([body,adjacent,recheck,horizon,eligible_adjacent,guard_adjacent,eligible_approach,guard])+'\n')
print(json.dumps(dict(before_sources=CHANGED,module=str(out.relative_to(ROOT)),
                     sha256=hashlib.sha256(out.read_bytes()).hexdigest())))

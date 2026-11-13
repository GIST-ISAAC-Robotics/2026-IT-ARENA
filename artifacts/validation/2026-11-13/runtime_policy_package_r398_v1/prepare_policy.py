"""선택 정책의 원본을 보존하며 패키지 경로만 재배치한다. 생성 이후 실행 중 재생성 금지."""
from pathlib import Path
import hashlib
import json
import re

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
A = BASE.parent
OLD = A.parent/'2026-11-12'
PACKAGE = BASE/'staged/src/arena_autonomy/arena_autonomy/race_runtime_policy'
ledger = {}


def write(source, name, transform=lambda text: text):
    raw = source.read_bytes()
    text = transform(raw.decode('utf-8').replace('\r\n', '\n'))
    target = PACKAGE/name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding='utf-8', newline='\n')
    ledger[name] = dict(source=source.relative_to(ROOT).as_posix(),
                       source_sha256=hashlib.sha256(raw).hexdigest(),
                       staged_sha256=hashlib.sha256(target.read_bytes()).hexdigest())


def prior(text, alias, source):
    for pattern in (r'^import importlib.util\n', r'^from pathlib import Path\n', r'^import sys\n'):
        text = re.sub(pattern, '', text, flags=re.M)
    start = re.search(r'^(?:A|HERE)\s*=\s*Path\(__file__\)', text, re.M).start()
    end = text.index('\n', text.index('spec.loader.exec_module(', start))
    return text[:start]+f'from . import {source} as {alias}\n'+text[end+1:]


def geometry(text):
    old = "    if (not current['timing_verified'] or not source_valid(scene)\n            or abs(scene.t_ref_s-now)>1e-9):"
    new = """    estimated = (current.get('timing_policy') == 'host_estimated_v1'
                 and getattr(history, 'scan_time_policy', None) == 'host_estimated_v1'
                 and raw['measurements']['time_basis'] == 'unverified_estimate')
    if (not (current['timing_verified'] or estimated) or not source_valid(scene)
            or abs(scene.t_ref_s-now)>1e-9):"""
    assert text.count(old) == 1
    return text.replace(old, new)


if __name__ == '__main__':
    PACKAGE.mkdir(parents=True, exist_ok=True)
    for source in sorted((OLD/'retained_route_execution_r266_v1/candidate_recheck').glob('*.py')):
        write(source, 'retained_recheck/'+source.name, geometry if source.name == '_geometry.py' else lambda t:t)
    write(OLD/'retained_route_execution_r266_v1/candidate_policy.py', 'observation_policy.py',
          lambda t:t.replace('import candidate_recheck as common', 'from . import retained_recheck as common'))
    write(OLD/'current_region_policy_r281_v1/candidate.py', 'current_region.py',
          lambda t:t.replace('import candidate_policy', 'from . import observation_policy as candidate_policy'))
    for folder, filename, alias, dependency in (
        ('current_solo_low_r343_v1','solo_low.py','base','current_region'),
        ('current_solo_gap_r346_v1','solo_gap.py','prior','solo_low'),
        ('solo_crawl_retry_r352_v1','solo_crawl.py','prior','solo_gap'),
        ('current_corner_r370_v1','solo_relative.py','prior','solo_crawl'),
        ('released_current_r375_v1','released_relative.py','prior','solo_relative')):
        write(A/folder/'candidate.py', filename, lambda t,a=alias,d=dependency:prior(t,a,d))
    write(A/'released_crawl_r361_v1/candidate.py', 'released_plan.py')
    write(OLD/'terminal_boundary_r278_v1/candidate.py', 'terminal_plan.py')
    (PACKAGE/'__init__.py').write_text('"""명시 선택형 운용 계산. 물리 출력 권한은 별도다."""\n', encoding='utf-8', newline='\n')
    (PACKAGE.parent/'__init__.py').write_text('from pkgutil import extend_path\n__path__ = extend_path(__path__, __name__)\n', encoding='utf-8', newline='\n')
    (BASE/'source_ledger.json').write_text(json.dumps(ledger,indent=2)+'\n', encoding='utf-8', newline='\n')
    print('prepared', len(ledger), 'source relocations; common source unchanged')

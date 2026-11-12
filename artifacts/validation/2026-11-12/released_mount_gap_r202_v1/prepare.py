"""새 실패 입력에 기존 장착 구조 인접 공백 검사를 대조할 사본만 만든다."""
import hashlib
import json
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
source=ROOT/'src/arena_autonomy/arena_autonomy/released_body_gap.py'
text=source.read_text('utf8')
text=text.replace('from arena_autonomy.current_path_recheck._gaps import bounded_body_nonreturn',
    'from arena_autonomy.current_path_recheck._gaps import bounded_body_nonreturn, bounded_adjacent_structure\nfrom arena_autonomy.known_structure import parse_sectors')
old='''        proof = bounded_body_nonreturn(scene, scan, queries, context['padding'],
                                      cfg.length_m, cfg.width_m, speed, target)'''
new='''        if cfg.known_structure_policy == 'structure_aware_v1':
            proof = bounded_adjacent_structure(scene, scan, queries, context['padding'],
                cfg.length_m, cfg.width_m, speed, target, parse_sectors(cfg.known_structure_sectors_json))
        else:
            proof = bounded_body_nonreturn(scene, scan, queries, context['padding'],
                                          cfg.length_m, cfg.width_m, speed, target)'''
assert text.count(old)==1
text=text.replace(old,new)
candidate=A/'candidate.py'
with candidate.open('x',encoding='utf8') as f:f.write(text)
with (A/'sources.json').open('x') as f:
    json.dump(dict(before=hashlib.sha256(source.read_bytes()).hexdigest(),
        candidate=hashlib.sha256(candidate.read_bytes()).hexdigest()),f,indent=2)

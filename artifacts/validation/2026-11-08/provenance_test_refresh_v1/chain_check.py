"""M3_SUCCESSORS 연결과 평가 스크립트 등록 위치를 읽기 전용으로 확인한다."""
import hashlib, json
from test_source_provenance_r0_plus_b3 import M3_SUCCESSORS as S
prev = 'config/tests/m3_execution_r0_sha256.json'
for n in S:
    c = json.load(open(n, encoding='utf-8'))
    if not (c['parent_contract'] == prev and hashlib.sha256(open(prev, 'rb').read()).hexdigest() == c['parent_contract_sha256']):
        print('BREAK', n, c['parent_contract'])
    for k in ('scripts/validate_g1_traffic.py', 'scripts/probe_braking_contact.py'):
        if k in c.get('approved_evaluation_scripts', {}):
            print('SCRIPT', n, k)
    prev = n
print(len(S), 'checked')

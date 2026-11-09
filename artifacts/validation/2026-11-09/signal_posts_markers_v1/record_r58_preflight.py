"""C4 실행 전 근거/진행 상태를 갱신한다. 과거 결과는 보존한다."""
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
p=A/'README.md';s=p.read_text(encoding='utf8')
s+='\nC2/C3의 올바른 노면 입력으로 재집계한 결과 기존 gates·metrics와 차이는 없었으며 실패 판정도 그대로다(`stationary_surface_reference_correction.json`). r58 출처 포함191검사와 관련지지49검사를 통과했다. C4는 기존 C3 조건에 `g1_narrow_occluder_bridge:=true`를 더해 실행하며, 모든 런타임 소스·시설·평가 코드를 종료까지 동결한다.\n'
p.write_text(s,encoding='utf8')
p=R/'docs/PROJECT_CONTEXT.md';s=p.read_text(encoding='utf8')
start=s.index('갱신:');end=s.index('\n\n목표는',start)
s=s[:start]+'''갱신: **2026-11-09 진행 중 — 신호·마커 지주와 마커 누락 대비.** Claude 5시간 경고 뒤 GPT가 구현하고 GPT-6 Luna가 실행한다. **신호등 양옆과 갈림길 마커 지주는 설치 확정**, 치수·발판·배치는 일부 임시다. P22 지름길은 제외하고 P21만 후속 검토한다. 신호/마커지주와 ID20/30미검출을 함께 넣은 E1은 **46.65m·29.917초·접촉0·STOP/정상종료**로 본선 한랩을 확인했다. 정지차 우회 C3는 상대를 지났지만 지주에 끊긴 벽 기준 때문에 복귀 중 정체했다. r58은 작은 지주 양옆의 실제 벽 반환을 연결하되 지주 점유를 유지한다. 저장1,028판단의 기준불일치0·0명령647→10, 관련191+49검사 통과 후 **C4 실제1회 검증 중**이다. 이전 평가기의 노면 입력 참조 오류는 실행별 자료로 바로잡았으며 C2/C3 수치·실패 판정은 불변이다. [실험 근거·재개점](../artifacts/validation/2026-11-09/signal_posts_markers_v1/README.md).

기존 성과·미완료는 아래 표와 [활성 작업표](track/V2026_11_06_MIGRATION_2026_11_07.md#현재-재개-지점)를 따른다. 이 시설 묶음의 대표가 확보되면 **지속 코너 병렬·빠른 추월·단독 속도 유지·P21·Jetson/실물**의 남은 작업으로 돌아간다. 과거 예약·종료 기록은 현재 작업 지시를 덮지 않는다.'''+s[end:]
p.write_text(s,encoding='utf8')
p=R/'docs/track/V2026_11_06_MIGRATION_2026_11_07.md';s=p.read_text(encoding='utf8')
old='C1/C2는각1.717/1.743m·최장정지14.354/14.662초로우회실패. 접촉0·최대두바퀴잔디·STOP/종료·소스불변. 근접 면을분리한r56도스캔후반의팽창으로반복0명령이남았다. 실제관측창을후방면과같은.8초/9스캔으로늘린r57은C2저장입력0명령244→68. 같은장면C3주행으로확인한다.'
new='C1/C2 반복정지 뒤 r57/C3는 상대 통과까지 진행했지만2.264m·최장정체11.726초로 복귀 실패. r58은 현재 벽의 좁은 지주 가림을 연결하며 C3저장1,028판단 기준불일치0·0명령647→10을 확인했다. 관련191+49검사 뒤 C4를 같은장면에서1회 검증 중이다. 점유·독립보호·STOP·판정선은 유지한다.'
assert old in s;s=s.replace(old,new);p.write_text(s,encoding='utf8')
p=R/'docs/DEVELOPMENT_PLAN.md';s=p.read_text(encoding='utf8')
s=s.replace('지주 옆 정지차 C2는 반복0명령으로 정체해 원인을 보완한다.',
    '지주 옆 정지차 C3는 통과 뒤 본선 복귀가 남았고, 현재 벽 기준의 가림을 보완한 C4를 확인한다.')
p.write_text(s,encoding='utf8')
print('r58 preflight recorded')

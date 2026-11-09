"""마커 지주·미검출 동시 대표 결과를 현황에 반영한다."""
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
p=A/'README.md';s=p.read_text(encoding='utf8')
s=s.replace('설치 확정된 마커 지주까지 넣은 E1 대표를 진행한다.', '설치 확정된 마커 지주까지 넣은 E1도 본선1랩을 마쳤다.')
s+='''
## E1: 신호·마커 지주와 마커 누락 동시 대표

`e_all_posts_blank`는46.65 m·1랩·출발 후29.917초·최고2.8475 m/s, 진행>.5m 이후<.05m/s 최장0.092초, 본선중심 최대오차0.1472m·접촉0으로 원report passed=true다. ID20/30은 숨김 적용·미검출,0/45만 관측했다. P21 본선 구간 최저1.146m/s·2.212초이며 P22는 지름길 진입이 아닌 본선 구간이다.

실행 월드에 마커 지주 모델2개가 실제 삽입됐고 지정 SDF와 XML이 같았다. 주행829표본에서 차체-마커지주 최소0.2412m·발판0.2216m·겹침0, 신호지주는0.2888/0.2763m다. STOP716표본도 지주/발판 겹침0, 정착·변위0.9412m·정상종료·소스불변이다. 고속 STOP변위의 기존 한계는 해결하지 않았다. `e1_summary.json`, `e_all_posts_blank/marker_post_evidence.json`을 따른다. 시설 치수/위치는 임시이며 이 한 번의 단독 대표를 교통·실물 전체 검증으로 확대하지 않는다.
'''
p.write_text(s,encoding='utf8',newline='\n')
p=R/'docs/track/V2026_11_06_MIGRATION_2026_11_07.md';s=p.read_text(encoding='utf8')
s=s.replace('기존 저장 스트레스 0속도15→65(OFF)/51(ON) 보존. 실제 추가 형상·주행은 준비 중',
 'E1 대표 성립: 신호+마커지주·마커20/30가림으로46.65m·29.917초·최장저속0.092초·접촉0·STOP/정상종료·소스불변. 지주 최소0.2412m·발판0.2216m·겹침0. ID20의 도로 위 가상지주 배치는 폐기하고 원판을 유지한 접근측 잔디 지지틀 사용(실물규격 아님). [배치·결과](../../artifacts/validation/2026-11-09/signal_posts_markers_v1/README.md)')
s=s.replace('**현재 묶음:** [고속 STOP 기록 분석]', '**11/9 지주 작업 전 묶음(보존):** [고속 STOP 기록 분석]')
s=s.replace('단독속도·빠른추월·운영·P21/P22·최신Jetson/실물 목록은 유지한다.', '단독속도·빠른추월·운영·P21·최신Jetson/실물 목록은 유지한다(P22는 위 후속 결정으로 제외).')
p.write_text(s,encoding='utf8',newline='\n')
p=R/'docs/PROJECT_CONTEXT.md';s=p.read_text(encoding='utf8')
s=s.replace('단면·위치는 아직 미정이다. [실패·원인·재개 상태]',
 '단면·위치는 아직 미정이다. 임시 마커지주까지 넣은 E1도46.65m·29.917초·최장저속0.092초·접촉0·STOP/종료를 확인했다. [실패·원인·재개 상태]')
p.write_text(s,encoding='utf8',newline='\n')
print('E1 recorded')

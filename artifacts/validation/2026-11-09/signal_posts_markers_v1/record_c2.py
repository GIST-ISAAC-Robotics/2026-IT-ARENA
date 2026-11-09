"""실패한 C2를 원본 보존한 채 활성 현황에 반영한다."""
from pathlib import Path
A=Path(__file__).resolve().parent; R=A.parents[3]
text='C2 r56 실제 주행도1.743m·최장정지14.662초·우회 미완료다. 접촉0·최대두바퀴잔디·STOP/정상종료·소스불변은 확인했다. 독립 보호층은 clear였고 우회 출력이 스캔 사이0으로 반복돼, 차체 걸림보다 현재 반환의 시간 팽창을 우선 분석한다.'
p=A/'README.md';s=p.read_text(encoding='utf8')
s=s.replace('C의 지주 앞 정지차 정체는 r56 비교 뒤 실제주행 대기이며', 'C의 지주 앞 정지차 정체는 r56 실제 C2에서도 미해결이며')
s+='\n## C2: 근접 면 분리 후에도 남은 스캔 사이 정지\n\n'+text+'\n\n`c_stationary_posts/run2` 원report failed를 보존한다. 원시 접촉·평가 겹침0, 최소차량간격0.12309m, 최종BYPASS다. STOP변위0.002145m·정착0.626초다. 실제 지주 위치에 도달하지 않아 지주 통과 성공이 아니다. `dense_post_clearance.json`의 실제 실행 월드 지주/발판 평가 간격은0.7658/0.7479m 이상이다.\n\n14초 이후 독립 보호133표본은 모두clear다. 예를 들어24.12–24.16초 우회기는0.35m/s를 내지만24.18–24.20초에는 `known_occupied_command_hold`(clearance0)로 바뀌고24.22초 다시0.35를 낸다. 실제 발행은 가속 제한 때문에0→0.02→0.04→0.06→0.08 부근에서 반복돼 차체는약0.02–0.04m/s만 진행한다. 원시 반환·고정 간격을 없애는 방식 대신, 어떤 면의 오래된 측정 여유가 현재 차체에 겹치는지 분리한다.\n'
p.write_text(s,encoding='utf8')
p=R/'docs/PROJECT_CONTEXT.md';s=p.read_text(encoding='utf8');s=s.replace('C 정지차 정체는 근접 관측면별 운동 창 r56을 연결해 비교 중이다.',text);p.write_text(s,encoding='utf8')
p=A/'OPERATING_NOTES.md';s=p.read_text(encoding='utf8');s=s.replace('C 정지차 우회는 r56 실제 결과를 확인한다.', 'C 정지차 우회는 r56/C2도 실패했다. 현재 분석은 반복0명령의 측정 나이·관측면별 팽창 원인이다.');p.write_text(s,encoding='utf8')
p=R/'docs/DEVELOPMENT_PLAN.md';s=p.read_text(encoding='utf8');s=s.replace('지주 옆 정지차 C와 마커 지주까지 포함한 E를 우선 마친다.', '마커 지주까지 포함한 E도 본선1랩을 마쳤다. 지주 옆 정지차 C2는 반복0명령으로 정체해 원인을 보완한다.');p.write_text(s,encoding='utf8')
print('C2 failure and next diagnosis recorded')

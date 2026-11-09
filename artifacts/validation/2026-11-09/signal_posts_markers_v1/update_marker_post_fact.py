"""11/9 후속 설치 확정과 남은 규격 가정을 현재 질의 문서에도 반영한다."""
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
p=R/'docs/HARDWARE_AND_RULE_QUESTIONS.md';s=p.read_text(encoding='utf8')
old='- **지지 기둥 — 확인 종료(11/2 사용자 전달): 실제 트랙 안에는 기둥이 없다.** 11/3 팀 파생본에서 `tl_post` 시각·충돌을 함께 제거했다. 공식 원본·기둥 포함 실패 이력은 보존한다. [기하/주행 결과](../artifacts/validation/2026-11-03/post_free_facility_v1/README.md). 사용자 전달 확인과 공식 저장소 수정 여부는 구별하며 추가 기둥 질문은 하지 않는다.'
new='- **지지 기둥 — 11/9 사용자 최신 전달:** 신호등 양옆과 갈림길 마커 쪽 모두 지주 설치가 확정됐다. 신호 제작측 STL의 지주·발판은 확보했으나 도로 45 cm 시절 자료여서 61 cm 간격은 임시 환산이다. 갈림길 지주의 개수·단면·발판·위치는 아직 미정이다. ID20 판 직하 양옆 배치는 코너 출구 도로에 겹치므로, 현재 시험은 판 위치를 유지하고 잔디 위 접근측 지지대로 받치는 별도 가정을 쓴다. 최종 배치 사진/치수 수신 시 대체한다. [현재 작업·배치 근거](../artifacts/validation/2026-11-09/signal_posts_markers_v1/README.md). 11/2 기둥 없음 전달과 [11/3 제거 결과](../artifacts/validation/2026-11-03/post_free_facility_v1/README.md)는 당시 이력으로 보존한다.'
assert old in s;s=s.replace(old,new)
s=s.replace('새 원본을 보존하고 11/3 코스 안 기둥만 제거한 팀 파생본을 적용했다.', '11/6 트랙 변경은 [새 코스 감사](track/OFFICIAL_V2026_11_06_AUDIT.md)를 추가로 따른다. 11/3 기둥 제거 파생본은 당시 이력이며, 11/9 신호·마커 지주 설치 확정은 아래 최신 항목을 따른다.')
p.write_text(s,encoding='utf8',newline='\n')
p=R/'artifacts/validation/2026-11-09/signal_post_source_v1/README.md';s=p.read_text(encoding='utf8')
s += '\n**후속 확인(11/9):** 위 표의 갈림길 설치 여부는 당시 미확정 상태다. 이후 사용자가 마커 쪽 지주도 설치 확정이라고 전달했다. 신호 STL은 마커 지주의 실제 설계 근거가 아니며, 단면·개수·위치는 별도 미정이다. [현재 임시 배치·검증](../signal_posts_markers_v1/README.md)을 따른다.\n'
p.write_text(s,encoding='utf8',newline='\n')
p=A/'README.md';s=p.read_text(encoding='utf8')
s += '''
## 갈림길 임시 시설의 배치 근거(E1 준비)

`marker_post_layout_audit.json`은 ID20 판(x6.155,y6.0)이 직각 코너 위에 있으므로 같은 단면 ±0.3725m 중 남쪽 지주·발판이 도로와 전부 겹친다는 것을 확인했다. 앞선 저장 재생 E의 배치도 이 가정이었다. 실패 수치를 현실 시설 결함으로 해석하지 않는다. 새 `marker_posts_provisional.sdf/json`은 ID20 지지틀만 접근쪽으로0.3725m 옮기고 상부 지지대로 기존 판을 받친다. ID30은 판 직하 좌우 잔디에 둔다. 네 지주와 네 발판은 모두 잔디 위·도로/벽 겹침0이다. 판·원본 월드·노면/물성은 불변이며, 수량·15×14.62mm단면·40×50mm발판·위치는 임시다. 특히 실물 강성/시공 검증이 아니다.

기존 `evaluation_actor_sdf`로 실행 사본에 넣으며 주행 제어에는 반환/영상 외 시설 정답을 주지 않는다. E 평가에는 네 지주를 충돌 다각형에 추가하고 낮은 발판은 별도2D겹침으로 기록한다. 시험용 시설 삽입/높이/원본보존2검사 통과. E1은 D3 조건+마커 지주+20/30가림으로1회 시행하며 실제 삽입 XML과 최소간격/STOP/종료를 별도 확인한다.

**r56 재생 도구 오류(보존):** 첫 연결 비교는 baseline의 `_padding`만 옛 메서드로 바꾸고 생성 당시 보관한 `padding_provider`를 바꾸지 않아 두 제어기가 모두 새 후보를 썼다. C968/r18 749불일치·명령변경0 결과는 `replay_r56_instrumentation_failure/`에 보존했다. provider도 교체한 재생으로 원기록 일치 여부를 다시 확인한다. 실행기 오류와 제어기 성능을 혼동하지 않는다. 새 회귀의 tuple.copy 검사 실수1건은 numpy 사본으로 수정했고 관련34검사 통과; 전체 출처 포함165통과는 별도 원 로그에 있다.
'''
p.write_text(s,encoding='utf8',newline='\n')
print('latest marker-support facts recorded')

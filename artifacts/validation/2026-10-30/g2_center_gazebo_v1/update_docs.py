"""현황 문서의 해당 항목만 교체한다. 일회성 문서 편집 도구."""
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
REPORT='../artifacts/validation/2026-10-30/g2_center_gazebo_v1/README.md'
LINK='../artifacts/validation/2026-10-30/g2_corridor_connection_v1/README.md'
HANDOFF='../artifacts/validation/2026-10-30/g2_center_gazebo_v1/HANDOFF.md'


def edit(path,pairs):
    target=ROOT/path
    text=target.read_bytes().decode('utf-8')
    for old,new in pairs:
        if text.count(old)!=1:
            raise ValueError(f'{path}: {old[:50]}')
        text=text.replace(old,new)
    target.write_bytes(text.encode('utf-8'))


edit('docs/PROJECT_CONTEXT.md',[
 ('갱신: **2026-10-30 (KST), 후방 확인 기억 연결 후 G2 중앙 통로 성립 분석과 오프라인 후보**.',
  '갱신: **2026-10-30 (KST), 후방 확인 기억 연결 후 G2 중앙 통로 후보의 명시 시험 연결과 Gazebo 1회**.'),
 ('중앙 표적 정체는 [오프라인 합성 후보](../artifacts/validation/2026-10-30/g2_inner_boundary_v1/README.md)까지만 진전했다.**',
  f'중앙 표적은 명시 시험 옵션의 [Gazebo 1회]({REPORT})를 통과했으나 복합 오차 없는 한 실행이고 기본값은 그대로다.**'),
 ('[10/30 오프라인 후보](../artifacts/validation/2026-10-30/g2_inner_boundary_v1/README.md)는 합성 처음 쓰는 시드 8/8 통과·복귀·접촉0이나 운영 미연결·새 가정이며 왼쪽 통로 미성립, 통과 중 측방 침범 시 접촉이 있다.',
  f'[10/30 명시 OFF 시험 옵션 `corridor_v1`]({LINK})의 [Gazebo 1회]({REPORT})는 오른쪽 통과 5.06 s·복귀·접촉0·최소 표적 128 mm·벽 64 mm·바퀴≤2·STOP·1041판단 동일이다. 복합 오차를 넣지 않은 1회이며 새 가정이다. 합성에서는 왼쪽 통로 미성립, 통과 중 측방 침범 접촉, 관측 조건 0.225 m/s의 큰 지연이 남는다.'),
 ('**현재 실행 상태(10/30 20:20 KST 전후):** Claude Opus 5.5가 중앙 통로 설계 묶음의 오프라인 비교를 마쳤고 Claude Haiku 5.5가 실행·회수를 맡았다. 합성 시험은 모두 종료됐다. 운영 코드는 바꾸지 않았고, 시험 연결 범위는 GPT 주 에이전트의 검토를 기다린다. 사용자 중지 전까지 계속 개발하며 5시간 한도 경고 때 GPT가 이어받는다.',
  f'**현재 실행 상태(10/30 20:40 KST 전후):** Claude Opus 5.5가 중앙 통로 후보의 명시 연결과 Gazebo 1회를 마쳤고 Claude Haiku 5.5가 실행·회수를 맡았다. 시험 프로세스는 모두 종료됐다. Claude 5시간 90% 경고로 이후 작업을 [인계]({HANDOFF})하며 GPT가 이어받는다. 사용자 중지 전까지 계속 개발한다.'),
 ('Gazebo·실물·Jetson은 미검증이며 운영 연결은 없다.',
  f'이후 주 에이전트의 범위 결정으로 [명시 기본 OFF 연결]({LINK})을 하고, 등록값 0.225 m/s·3 m/s²·정렬 30 mm로 [Gazebo 1회]({REPORT})를 통과했다. 관련 회귀 277·오프라인↔런타임 3,093호출·기존 schema4 재생을 확인했고 전체 회귀는 실행하지 않았다. 이번 1회에서 편측 헛돎은 나타나지 않았지만 해결로 쓰지 않는다. 기본값·경기 채택·실차 위험 수용은 아니며 복합 오차 Gazebo·반복성·실물·Jetson은 미검증이다.'),
])
edit('docs/DEVELOPMENT_PLAN.md',[
 ('갱신: **2026-10-30 (KST), 후방 확인 기억 연결과 제한 주행, G2 공간 예약 대조, 중앙 통로 성립 분석과 오프라인 후보**.',
  '갱신: **2026-10-30 (KST), 후방 확인 기억 연결과 제한 주행, G2 공간 예약 대조, 중앙 통로 후보의 명시 연결과 Gazebo 1회**.'),
 ('는 여유 계산을 일차 원인으로 분리했고 합성 처음 쓰는 시드 8/8 통과·복귀를 보였다. 다만 새 가정·새 노출이 있어 운영 연결 전에 범위를 결정한다. **다음 순서는 ① 시작 가속 항과 30 mm 정렬을 제한 시뮬레이션 가정으로 받을지 판단, ② 받는다면 명시 기본 OFF G2 시험 경로에 연결해 Gazebo 중앙 표적 제한 주행, ③ 실물 C1의 면별 산포·공통 치우침·회전 방향과 시작각 측정, ④ 측방 침범에서 옆 정지 대신 전진 회피가 유리한 조건 비교**다. 왼쪽 통로 미성립과 G1→G2 자동 전환의 근거 창 시점도 남는다.',
  f'는 여유 계산을 일차 원인으로 분리했다. [명시 OFF 시험 옵션]({LINK})을 연결했고 복합 오차 없는 [Gazebo 1회]({REPORT})가 통과했다. 기본값·경기 채택은 아니다. **다음 순서는 ① Gazebo 복합 오차 주입 가능 여부 확인과 새 등록, ② 합성 0.225 m/s 지연의 대응 판단, ③ 측방 침범에서 옆 정지 대신 전진 회피가 유리한 조건 비교, ④ 실물 C1의 면별 산포·공통 치우침·회전 방향과 시작각 측정**이다. 왼쪽 통로 미성립과 G1→G2 자동 전환의 근거 창 시점도 남는다. 같은 조건을 반복하지 않는다. [인계]({HANDOFF}).'),
])
activity=ROOT/'docs/activity/2026-10-30.md'
text=activity.read_bytes().decode('utf-8')
if '## G2 중앙 통로 명시 연결과 Gazebo 1회' in text:
    raise ValueError('activity section exists')
text=text.rstrip('\n')+'''

## G2 중앙 통로 명시 연결과 Gazebo 1회

- **활동:** GPT 주 에이전트의 범위 결정에 따라 Claude Opus 5.5가 후보를 전역 치환 없는 명시 기본 OFF 옵션으로 연결했고, Claude Haiku 5.5가 중앙 표적 Gazebo 1회를 실행했다. 등록값은 관측 조건 항 0.225 m/s, 시험 가속 3 m/s², 정렬 상한 30 mm다.
- **결과:** 관련 회귀 277과 출처 시험, 오프라인↔런타임 3사례 각 1031호출, 기존 schema4 재생을 확인했다. Gazebo는 등록 G2 판정을 통과했다. 오른쪽 통과 초록 후 5.06 s·중앙 복귀·접촉 0·최소 표적 128.3 mm·벽 64.0 mm·도로 밖 바퀴 ≤2·STOP·정상 종료·1041판단 동일이었다. 이번 1회에서 편측 헛돎은 나타나지 않았다.
- **한계·후속:** 복합 오차를 넣지 않은 1회이며 기본값·경기 채택·실차 위험 수용이 아니다. 원래 G1 판정 실패는 보존했다. 왼쪽 통로, 측방 침범, 치우침, 복수 차량, 자동 전환은 비검증 범위다. schema5 재생 불일치는 연결 전과 같고 전체 회귀는 실행하지 않았다. Claude 5시간 90% 경고로 GPT에 인계했다. 커밋·푸시·외부 게시·Jetson/전원 변경은 없다.
- **근거:** [Gazebo 결과](../../artifacts/validation/2026-10-30/g2_center_gazebo_v1/README.md), [실행 전 등록](../../artifacts/validation/2026-10-30/g2_center_gazebo_v1/DRIVE_PLAN.md), [연결](../../artifacts/validation/2026-10-30/g2_corridor_connection_v1/README.md), [인계](../../artifacts/validation/2026-10-30/g2_center_gazebo_v1/HANDOFF.md).
'''
activity.write_bytes(text.encode('utf-8'))
print('updated')

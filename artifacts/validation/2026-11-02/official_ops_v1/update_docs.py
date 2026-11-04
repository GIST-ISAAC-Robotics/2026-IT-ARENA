"""인계 시점 상태 문서 갱신. 치환 실패 시 중단. 재실행 금지."""
from pathlib import Path

A = Path('artifacts/validation/2026-11-02/official_ops_v1')
for name in ('README.md', 'HANDOFF.md'):
    path = A/name
    data = path.read_bytes().replace(b'\r\n', b'\n').decode('utf-8').replace('41개', '37개')
    path.write_bytes(data.encode('utf-8'))


def patch(path, pairs):
    target = Path(path)
    raw = target.read_bytes()
    crlf = b'\r\n' in raw
    text = raw.replace(b'\r\n', b'\n').decode('utf-8')
    for old, new in pairs:
        assert text.count(old) == 1, old[:70]
        text = text.replace(old, new)
    data = text.encode('utf-8')
    target.write_bytes(data.replace(b'\n', b'\r\n') if crlf else data)


L = '../artifacts/validation/2026-11-02/official_ops_v1/README.md'
patch('docs/PROJECT_CONTEXT.md', [
    ('갱신: **2026-11-02, 새 공식 월드 대표 교통 연결**.', '갱신: **2026-11-02, 최소 경기 운영 흐름 연결(인계 상태)**.'),
    ('| 운영 | [B3 CLI](autonomy/RACE_OPERATOR_RUNBOOK.md)의 상태·상위 STOP·새 경기·해제는 있음. **웹 모니터·같은 경기 복구 RESET/START·통합 사건 내보내기는 계획 단계**. [구체 설계](autonomy/RACE_MONITOR_AND_RECOVERY_PLAN_2026_11_02.md) |',
     f'| 운영 | 기본 OFF 후보 `race_ops` 웹 화면, 같은 경기 STOP/RESET/START, 재시작 복원, 사건 내보내기 연결. [대표 흐름 a6]({L}) 시뮬 통과. 공용망·실제 MCU·실제 정지 미검증. [사용법](autonomy/RACE_OPERATOR_RUNBOOK.md)·[설계](autonomy/RACE_MONITOR_AND_RECOVERY_PLAN_2026_11_02.md) |'),
    ('| [11/2 새 월드 전방 교통](../artifacts/validation/2026-11-02/official_forward_v1/README.md) |',
     f'| [11/2 경기 운영 흐름]({L}) | 최초 신호 없는 START 거부. 왼쪽 복구, ID30 안 보이는 오른쪽 복구, 프로세스 재시작 후 오른쪽 복구가 START 1회 뒤 시뮬 0.40–0.43초에 허가되고 평가 정답으로 본선 진행. 지름길 표본 0. 준비 중 STOP·연결 상실 취소, 응답 유실 1회 실행 | 단독 1회. 접촉 센서 미포함. 교통 후보 결합·실제 브라우저로 Gazebo 조작 미실행. Chrome 버튼은 대역 차량 확인. 재획득 옵션 ON 기준 |\n| [11/2 새 월드 전방 교통](../artifacts/validation/2026-11-02/official_forward_v1/README.md) |'),
    ('현 legacy_reset은 새 경기로 돌아가므로 세이브포인트의 같은 경기 복구와 구분해야 한다. [복구 설계](autonomy/RACE_MONITOR_AND_RECOVERY_PLAN_2026_11_02.md)는 최종 배치 뒤 입력으로 경로를 재계산하고 마커 없는 오른쪽 기준 구간의 본선 복구까지 확인하도록 보완했으며 아직 미구현·미검증이다. 심판 배치 후 추가 10초 대기는 규정이 아니다.',
     f'같은 경기 복구는 [대표 흐름]({L})에서 연결됐다. **남은 분기:** 왼쪽 상태로 멈춘 차를 ID30을 보지 못한 채 그 너머로 옮기면 `side`가 왼쪽으로 남는다. 합류부 영향은 추측이며 미실행이다. 심판 배치 후 추가 10초 대기는 규정이 아니다.'),
    ('1. **C 운영 우선 제안:** 교통 기본 동작이 [대표 조건](../artifacts/validation/2026-11-02/official_traffic_v1/README.md)에서 이어졌으므로 비어 있는 상태 화면·같은 경기 STOP/RESET/START·사건 로그를 다음 필수 기능으로 제안한다. 교통 거리 시험의 새 월드 이전과 후보 기본 활성화 판단은 그 뒤 통합 점검에서 본다.',
     f'1. **C 운영 이어가기:** 시뮬 필수 흐름은 [연결됐다]({L}). 미완료는 수정 뒤 전체 회귀, 마커 건너뜀 재배치 진단, 교통 후보 결합, 공용망·실제 MCU 확인이다. [인계와 재개 명령](../artifacts/validation/2026-11-02/official_ops_v1/HANDOFF.md)을 따른다. 교통 거리 시험의 새 월드 이전과 후보 기본 활성화 판단은 그 뒤 통합 점검에서 본다.'),
])
patch('docs/autonomy/RACE_MONITOR_AND_RECOVERY_PLAN_2026_11_02.md', [
    ('**상태: 설계만 작성, 아직 구현하지 않음.** 11/1 최종 회의와 사용자 요청을 반영한다.',
     '**상태: 첫 구현 연결, 시뮬 대표 흐름 통과([결과](../../artifacts/validation/2026-11-02/official_ops_v1/README.md)). 기본 OFF 후보이며 공용망·실물 미검증.** 11/1 최종 회의와 사용자 요청을 반영한다.'),
    ('아래 버튼·화면을 기존 도구에서 이미 쓸 수 있다는 뜻이 아니다.', '아래 표는 설계 기준이다. 실제 구현 범위는 결과 문서를 따른다.'),
])
patch('docs/autonomy/RACE_OPERATOR_RUNBOOK.md', [
    ('> **11/2 후속 계획:** [모니터·세이브포인트 복구 RESET/START](RACE_MONITOR_AND_RECOVERY_PLAN_2026_11_02.md)를 설계했다. 아래는 **현재 구현된 CLI**의 사용법이며 웹 화면이나 같은 경기 이력 초기화가 추가됐다는 뜻이 아니다. 현 `new-race`/`legacy_reset`은 출발 사실을 지우므로 세이브포인트 복구용으로 혼용하지 않는다.',
     """> **11/2 후속:** 웹 화면과 같은 경기 복구가 [기본 OFF 후보로 연결됐다](../../artifacts/validation/2026-11-02/official_ops_v1/README.md). 운영은 아래 0절을 우선한다. `new-race`/`legacy_reset`은 출발 사실을 지우므로 세이브포인트 복구용으로 혼용하지 않는다. 옛 수동 번호 입력 지시가 복구 계획과 충돌하면 계획을 따른다.

## 0. 웹 화면으로 운영

차량에서 `race_ops`를 실행하고 노트북 브라우저로 연다. 공용 AP 접속·보안·실제 정지는 현장 미검증이다.

```bash
ros2 run arena_autonomy race_ops --ros-args -p vehicle_id:=car1 -p host:=0.0.0.0 -p port:=8765 \\
  -p token_path:=/path/token.txt -p record_path:=/path/race_record.json -p log_path:=/path/ops_log.jsonl
```

토큰 파일이 없으면 새로 만든다. 읽기 전용 접속은 `view_token_path`로 별도 토큰을 준다. `host` 기본값은 `127.0.0.1`이다.

| 상황 | 조작 |
|---|---|
| 프로세스 시작·새 경기 뒤 | **최초 출발 준비**: 위험 제거 확인 뒤 해제·ARM. 출발은 차량이 신호를 시각 인식해야 한다 |
| 경기 중 개입 | **STOP** → **복구 RESET** → 심판의 최종 배치 확인 → **재개 START** 한 번. RESET은 이동 전후 어느 쪽도 된다 |
| 제어 프로세스 재시작 뒤 | 화면의 저장 기록이 현재 경기임을 확인하고 **같은 경기 복구** → **재개 START** |

- 번호를 입력하지 않는다. 화면이 보여 준 경기·개입 번호가 버튼과 함께 묶이고, 달라졌으면 거절된다.
- START 수락은 주행 허가가 아니다. 화면이 `복구 준비`에서 `주행`으로 바뀌어야 허가다. 취소되면 이유가 표시되고 자동 재실행되지 않는다.
- 결과 미확인이면 다시 누르기 전에 상태를 확인한다. 화면은 자동 재전송하지 않는다. STOP은 언제든 다시 누를 수 있다.
- Wi-Fi STOP은 HW 정지를 대체하지 않는다. 화면의 실제 차체 정지는 항상 `미확인`이다. 모니터 연결이 끊겨도 정상 주행 차량은 멈추지 않는다. 다만 START 준비 중 끊기면 그 START는 취소된다.
- 왼쪽 상태로 멈춘 차를 ID30을 보지 못한 채 그 너머로 옮기는 재배치는 미검증이다.
- 사건 시간축: 화면의 CSV/HTML, 또는 `scripts/race_event_export.py --log ops_log.jsonl --request 요청번호 --output 출력경로`.

아래는 CLI 사용법이다. `recovery-reset`·`resume-start`도 같은 번호 규칙으로 쓸 수 있다."""),
])
patch('docs/DEVELOPMENT_PLAN.md', [
    ('| M4 경기 정지·재개 | 최초 시각 출발과 같은 경기 세이브포인트 재개 분리. 최종 배치 이후 입력으로 경로/기준 벽 재수립, STOP/RESET/START, 사건 기록 | C 운영 |',
     '| M4 경기 정지·재개 | [시뮬 대표 흐름 연결](../artifacts/validation/2026-11-02/official_ops_v1/README.md)(기본 OFF). 마커를 건너뛴 재배치, 교통 후보 결합, 공용망·실제 MCU 확인이 남음 | C 운영 |'),
    ('| **C. 최소 경기 운영(다음 제안)** | [기존 CLI 기반 설계](autonomy/RACE_MONITOR_AND_RECOVERY_PLAN_2026_11_02.md): 상태 화면 → STOP/같은 경기 RESET/START → 사건 로그를 연결. 최초 신호와 세이브포인트 재개 구분. 최종 배치 뒤 START 한 번으로 이전 입력을 끊고 새 관측에서 경로·기준 벽을 재수립 |',
     '| **C. 최소 경기 운영(시뮬 연결, 후속 남음)** | [대표 흐름 결과](../artifacts/validation/2026-11-02/official_ops_v1/README.md)·[인계](../artifacts/validation/2026-11-02/official_ops_v1/HANDOFF.md). 상태 화면·STOP/같은 경기 RESET/START·재시작 복원·사건 내보내기 연결. 남은 일: 전체 회귀, 마커 건너뜀 재배치 진단, 교통 후보 결합, 현장 확인 |'),
])
activity = Path('docs/activity/2026-11-02.md')
raw = activity.read_bytes()
crlf = b'\r\n' in raw
body = raw.replace(b'\r\n', b'\n')
add = """
## 최소 경기 운영 연결

- **한 일:** 같은 경기 복구 RESET/START, 입력 경계, 재시작 복원, 별도 프로세스 운영 어댑터·웹 화면, 사건 내보내기를 구현했다. 구현은 Claude Opus 5.5, 시뮬 실행은 Claude Haiku 5.5 보조가 맡았다.
- **결과:** 대표 흐름 a6가 37개 평가 관문을 통과했다. 왼쪽·ID30 안 보이는 오른쪽·프로세스 재시작 후 오른쪽 재개가 START 1회로 본선을 이어 갔다. Chrome 버튼 연결은 대역 차량으로 확인했다.
- **찾은 결함:** ARM 직후 DISARM과 합류부 영구 정체를 고쳤다. 후자는 기본 OFF `g1_standing_wall_reacquire`로 푼다.
- **한계·후속:** 5시간 사용량 경고로 GPT에 인계했다. 수정 뒤 전체 회귀, 마커 건너뜀 재배치, 교통 후보 결합, 공용망·실제 MCU는 미완료다.
- **근거:** [운영 결과](../../artifacts/validation/2026-11-02/official_ops_v1/README.md)·[인계](../../artifacts/validation/2026-11-02/official_ops_v1/HANDOFF.md).
""".encode('utf-8')
body = (body if body.endswith(b'\n') else body+b'\n')+add
activity.write_bytes(body.replace(b'\n', b'\r\n') if crlf else body)
print('updated')

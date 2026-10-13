# G1 판단 단계 비용과 Jetson 비교 준비

2026-10-13. 새 분리 경계 후보가 왼쪽 가림과 앞차 대기/재진행의 제한 회귀를 통과한 뒤, [판단 기록](../autonomy/G1_FRAGMENTED_BOUNDARY_2026_10_13.md)을 재사용했다. **Jetson 접속/실행은 하지 않았으며**, ROS·카메라·보호층·구동이 동시에 도는 부하 시험도 아니다.

## 1. 측정 계약

`profile_follow_trace.py`는 기록 순서/footer/손실 검사와 기존 출력의 전부 일치를 먼저 확인한다. 이후 워밍업 1회·측정 3회 동안 같은 리셋/입력 순서로 판단한다. 명령과 세부 사유가 하나라도 달라지면 비용 결과만으로 통과시키지 않는다.

포함한 단계는 기록된 운동 이력 복원, 현재 좌표로의 순차 LiDAR 투영, 지역 G1 명령 계산이다. **파일 읽기·JSON 파싱·출력 비교·ROS/DDS 콜백·영상·독립 보호·MCU/모터·실제 입력 도착과 시계 매핑은 제외**한다. 전체 제어 콜백보다 좁으며, 복원 단계는 오프라인 재생에만 있는 추가 비용이다. 20 ms는 50 Hz 주기의 참고선이지 새 합격/보호 한도가 아니다.

## 2. 최초 큰 값과 분리 과정

| 단계 | 왼쪽 / 앞차 초기 입력의 전체 p99, 최대 ms | 결론 |
|---|---|---|
| [v1 벽시계](../../artifacts/validation/2026-10-13/follow_profile_v1/README.md) | 18.773, 25.453 / 23.089, 28.336 | 일부 20 ms 초과. CPU 계산과 대기/메모리 회수를 구분 못함 |
| [v2 CPU·GC 관측](../../artifacts/validation/2026-10-13/follow_profile_v2/README.md) | 18.953, 28.050 / 22.613, 26.710 | 스레드 CPU도 비슷하고 긴 구간에 GC 포함. 전체 trace/출력 목록을 붙잡는 시험기 부담 가능성 |
| [v3 한 줄씩 읽기](../../artifacts/validation/2026-10-13/follow_profile_v3/README.md) | 8.841, 10.415 / 7.025, 7.900 | 같은 제어 코드에서 20 ms 초과 0. 시험기 보유 메모리의 영향 확인, 운영 GC 설정 미변경 |

v3의 중앙값은 각각 4.442/3.424 ms, 측정 호출은 1,122/1,656개였다. Python 3.12.3·NumPy 1.26.4의 WSL PC 결과다. 타 실행 간 스케줄/캐시 차이는 통제하지 않았으므로 수치를 제어 알고리즘의 정확한 개선율로 쓰지 않는다. v1/v2를 삭제하거나 실시간 실패로 확대하지 않는다. 모든 측정 호출에 GC가 관측됐고, GC 없는 그룹과의 비교는 없다.

**현재 결정:** 제어기 변경이나 GC 비활성화는 하지 않는다. 이 비용만을 이유로 새 최적화 작업을 계속 늘리지 않고, v3을 PC 판단 단계 기준선으로 남긴다. 전체 ROS 처리율/Jetson 비교에서는 여전히 영상·입력 지연·보호/출력·종료를 별도 확인해야 한다.

## 3. 최소 이관 묶음

기존 `prepare_jetson_bundle.py`에 `--follow-profile-only`와 반복 가능한 `--follow-trace`를 추가했다. 이 모드는 명시한 JSONL과 순수 Python/NumPy 의존 모듈만 담으며 ROS 노드·비밀번호·원본 bag 전체·빌드 산출물 전체를 수집하지 않는다. archive는 Git 제외 `build/`에만 저장하고 파일 목록/크기/SHA-256은 artifacts에 남긴다. 기존 묶음을 덮어쓰지 않는다.

`verify_follow_profile_bundle.py`는 archive와 각 멤버의 크기/해시·중복/위험 경로·링크 여부를 검사하고 추출하거나 전송하지 않는다. 이 검사는 내용 무결성이지 신뢰할 수 없는 archive의 실행 승인이나 Jetson 실측이 아니다. 포함한 테스트 파일은 소스 근거용이며 최소 묶음만으로 전체 pytest 환경이 제공되는 것은 아니다.

관련 회귀 34개와 22파일/6,680,245바이트 묶음의 해시 검사가 통과했다. 후속 **로컬 격리 추출본**에서 ROS overlay 없이 import·합성 캡처 감사·374/374 판단 재생을 확인했다. 원본/추출본의 입력과 소스는 불변이며 원격 전송/실행은 하지 않았다. 이것으로 PC 준비 묶음을 마감한다. 후속 감사기 변경은 생성 당시 archive에 소급 적용하지 않는다.

Jetson의 보존 환경은 Python 3.10.12·NumPy 1.21.5였으므로 두 신규 도구의 `hashlib.file_digest` 의존성을 제거했다. Python 3.10 문법/내부 의존성 검사와 3.11 API가 없는 조건의 회귀를 수행하지만 실제 ARM/패키지 실행 호환성은 아직 별도 확인 대상이다.

향후 사용자가 Jetson 실행을 요청하거나 해당 범위를 승인하면, 새 빈 작업 디렉터리에 묶음을 풀고 해시를 재확인한다. 기존 구동 환경을 덮어쓰거나 OS/전력 설정을 바꾸지 않는다. 최소 오프라인 실행 예시는 다음과 같다(실행할 파일은 manifest의 정확한 경로 사용).

```bash
PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts \
python3 scripts/profile_follow_trace.py artifacts/validation/2026-10-13/gapped_boundary_v3/side_left_s0/follow_decisions.jsonl \
  --input-mode stream --output artifacts/validation/NEW/jetson_side
```

이 명령에는 ROS/MCU·모터가 연결되지 않는다. 이것이 통과해도 다음 전체 ROS 재생에서는 입력 신선도·누적 적체·EOF 정지 출력·정상 종료를 다시 구분해야 한다. 원격 전송/설치/실행을 이번 문서로 자동 승인하지 않는다.

[A2/A3 인수인계](../reports/RECORD_REPLAY_JETSON_HANDOFF_2026_09_29.md), [묶음 검증 자료](../../artifacts/validation/2026-10-13/follow_profile_bundle_v1/README.md).

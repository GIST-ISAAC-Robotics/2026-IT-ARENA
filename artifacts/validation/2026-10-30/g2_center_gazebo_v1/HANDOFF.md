# 10/30 인계: G2 중앙 통로 연결과 Gazebo 1회

2026-10-30 약 20:40 KST. Claude 5시간 사용량 90% 경고로 GPT에 인계한다. 전체 개발 중단이 아니다.

## 적용 완료

- **소스:** `bypass_corridor_trial.py`(새 파일), `bypass_entry_trial.py`, `bypass_entry_follow.py`, `g2_entry_node.py`. 명시 `g2_corridor_policy=corridor_v1`일 때만 동작하고 기본은 `off`다.
- **시험·계약:** `tests/test_bypass_corridor_trial.py`, `tests/test_g2_corridor_provenance.py`, `config/tests/g2_corridor_r1_sha256.json`. `test_g2_handoff_provenance.py`, `test_source_provenance_r0_plus_b3.py`의 사슬 수정.
- **빌드:** `arena_autonomy` 설치 경로 대조 완료.
- **주행:** 중앙 표적 Gazebo 1회 완료, 등록 G2 판정 통과. [결과](README.md).
- **문서:** 후보·연결·Gazebo 보고서, 현황, 계획, 활동 기록 갱신. `.gitignore`에 원시 기록 제외 추가.

## 확인 완료와 미검증

- **확인:** 관련 회귀 277 + 출처 2, 오프라인↔런타임 3사례 각 1031호출, 기존 schema4 640·639호출 재생, Gazebo 판단 1041호출 재생.
- **미실행:** 저장소 전체 회귀. 범위는 관련 시험만이다.
- **기존 불일치:** schema5 재생 601/641, 603/656은 연결 전 소스도 같다. 원인은 다시 분해하지 않았다.
- **미검증:** 반복성, Gazebo 복합 오차, 왼쪽 통로, 측방 침범, 치우침, 복수 차량, 자동 전환, 실차, Jetson.
- **미분석:** 과거 왼쪽 시험의 헛돎이 이번에 나타나지 않은 이유.

## 프로세스

실행 중 시험 없음. Gazebo 잔존 0(`run1_residual_processes.txt`). 하위 Haiku 보조는 모두 완료했다. 커밋·푸시·외부 게시·Jetson/전원 변경 없음. 기존 미커밋 변경은 보존했다.

## 다음 판단

같은 조건을 반복하지 않는다. 우선순위 제안은 다음과 같다.

1. Gazebo 복합 오차 주입 가능 여부를 먼저 본다. 넣는다면 조건을 새로 등록한다.
2. 합성 0.225 m/s 지연의 원인을 후보 쪽에서 볼지, 오차 분포 실측을 기다릴지 정한다.
3. 측방 침범은 옆 정지와 전진 회피의 비교가 다음 설계다.
4. 실물 벤치에서 C1 면별 산포·공통 치우침·회전 방향·시작각을 잰다.

재현 명령: `artifacts/validation/2026-10-30/g2_center_gazebo_v1/run_trial.sh`는 `run1`이 있으면 실행을 거부한다. 새 실행에는 새 폴더와 등록이 필요하다.

# r498 공통 통합 결과

2026-11-14, Claude Opus 5.5. **r498 후보 3개 공통 파일과 신규 시험 1개를 원바이트로 공통에 반영했고 focused 회귀 93 passed(exit 0)를 확인했다.** 판단·TTL·경로·보호 조건과 기본 설치 r500은 바꾸지 않았다.

## 적용 절차

- 복사 전 공통 `traffic_solo_gap.py`·`local_follow.py`·`pass_release_trial.py`의 SHA가 [원본 manifest](../source_manifest.json), `original/` 사본, r501 `prepared.json`의 baseline과 모두 같음을 확인했다. `tests/test_threat_view_transfer.py`는 복사 전 공통에 없었다.
- `original/`↔`stage/` 소스 차이는 위 3파일뿐이며(`__pycache__` 제외) 내용은 r501 검토 대상 diff와 같다.
- `git apply` 대신 `stage/`의 원바이트를 복사했다(CRLF 유지). 복사 후 4파일 SHA가 r501 candidate SHA와 일치한다. 상세는 [receipt.json](receipt.json).

## 검사

- Windows Python에는 pytest와 `ackermann_msgs`가 없어 사전 고지 후 기존 WSL(ROS jazzy, python3.12) 환경에서 한 번 실행했다: [run_tests.sh](run_tests.sh) → `tests/test_threat_view_transfer.py tests/test_traffic_solo_gap.py tests/test_pass_release_trial.py` **93 passed**, exit 0, [로그](focused_pytest.log). r498의 80회귀와 파일 구성이 같은지는 기록이 없어 확인하지 못했다.
- 실행 시점에 다른 담당이 공통 `moving_pass.py`를 수정한 상태였다(이 작업에서 수정하지 않음). 해당 변경이 함께 로드된 상태의 결과다.
- ARM/Jetson 성능, 508 재생, ROS 노드·Gazebo·실차는 실행하지 않았다(r501 결과를 사용).

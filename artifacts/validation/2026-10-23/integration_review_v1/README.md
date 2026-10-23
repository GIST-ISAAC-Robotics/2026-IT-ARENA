# 10/23 입력 기록·투영 후보 통합 회귀

- `run_checks.sh` 고정 실행 1회, exit 0. **4,281 passed, 실패/skip 0**, pytest 벽시계 172.13 s.
- C++ 수신 API/프로덕션 API·수신·명령·링크·wire helper를 ASAN/UBSAN으로 빌드한 뒤 전체 `tests`를 실행했다. 각 binary 환경 변수를 지정해 해당 검사가 선택적으로 빠지지 않게 했다.
- `sources_before.sha256`/`sources_after.sha256` 차이 0 B. 당시 `src/scripts/tests/config` 스냅샷과 실행 바이너리는 로컬 보존·Git 제외, 로그/해시/재현 명령은 이 폴더에 둔다.
- GPT가 구현·최종 검토하고 GPT-6 Luna가 실행·대기·결과 회수를 담당했다. 실행 프로세스 잔존 없음.
- 원시 기록·독립 가속도·명시 점별 시각 후보를 더한 소스의 PC 회귀다. 실제 UART/카메라/C1·Jetson·전체 코스 또는 Gazebo 종료 성공을 뜻하지 않는다. 기존 실패·운영 미연결·실물 시간 관문의 상태는 그대로다.

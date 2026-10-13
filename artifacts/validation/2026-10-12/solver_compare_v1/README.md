# Dantzig/PGS 제한 대조 — 실행 전 범위

- 단일 평면·물리 1 ms·수동 제동 근사·20 ms 예약 적용·phase 10 ms·embedded paused·GC defer를 고정한다.
- `d1_dantzig`, `p1_pgs`를 각 1회 실행한다. 기본 제어기/launch/구동 플러그인/기본 solver는 수정하지 않는다.
- 이전 r7 runtime 소스·바이너리와 전체 probe 출처를 사전 검사한다. wrapper는 생성된 flat_world 사본에 `physics/dart/solver/solver_type`만 추가한다.
- gz-physics 7.6.0 `WorldFeatures::SetWorldSolver`는 setter 뒤 실제 `getBoxedLcpSolver()->getType()`을 출력한다. `simulation.log`의 이 로그로 primary 선택을 검증한다. 매 step의 Dantzig secondary fallback까지 계측하는 것은 아니다.
- 두 실행은 입력 유효·단일 모터축·실측 step·741개 정상 적용 동일·STOP ACK·실제 정지·정상 종료·잔존 0·소스/원시 무결성을 각각 확인한다. 실행 실패 시 다음 사례를 자동으로 밀어붙이지 않는다.
- 비교 지표는 기존 7개 제동 지표, 0명령 이후 0.22초 접촉별 방향 마찰 한계 비율, 근평면 충격량–운동량 잔차다. 마찰 비율 1 초과 여부는 관측 결과로 보고하며 입력 유효성 판정으로 숨기지 않는다. 기존 1e-5 rad 근평면 조건은 유지한다.
- 차체 정답은 평가에만 사용한다. 보호층 우회·마찰/토크 완화·실물 MD20A 재현 주장·PGS 기본 채택·G1 해결 주장을 하지 않는다.
- 상세 JSONL/i64는 기존 정책대로 로컬 보존하고 보고서/소스/해시를 추적한다.

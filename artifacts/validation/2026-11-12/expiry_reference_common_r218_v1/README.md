# 최근 경로 참조 공통 연결 r218

r216의 시험 전용 함수 교체/통신 wrapper를 공통 모듈과 명시 선택 옵션으로 옮겼다. `g1_expiry_route_reference_trial:=true`는 `g1_junction_continuity:=true`, process 계산, fresh native 제출이 필요하며 기본값은 false다. 평가 지연은 별도이며 실제 입력 시각/실차 운용을 승인하지 않는다.

원천 시각 만료로 계산 결과를 폐기했을 때만 다음 세대 첫 요청에 원인을 전달한다. 이때 이미 얻은 경로/선택 벽/기준 시각을 새 현재 관측과 비교할 자료로 남기고, 활성 경로·명령·추종/우회 상태는 지운다. 새 STOP/RESET·시계/센서 변경이 있으면 이 힌트도 지우고 전체 초기화한다. 새 경로 후보가 없는 경우 비교하지 않는 r216 수정과 기존 호길이 일치/기간/거리를 유지한다.

## 확인한 결과

- 공통 ON: r212의466판단과 r217의473판단, 총939개 전체 비교 출력이 원 r216 prefix와 같다. r217의 실제 기록473개와도 같다.
- 공통 OFF: r217 입력473개 전체 비교 출력이 변경 전 기본 구현과 같다. 공통화 뒤 기본값이 바뀌지 않았다.
- 분리 사본의 실제 spawn·원천 만료/STOP·새 입력 경계 관련40검사 통과. 적용할 때 변경 전6파일 해시를 다시 확인했으며 검증 사본과 적용7파일 바이트가 같다.
- 공통 노드/launch·전체 출처 연결을 포함한254검사, 갈림길 우선순위8검사와2패키지 빌드11.7초를 통과했다. 주행은 앞선 r217의 제한 결과이며 공통 실행은 후속 시험으로 분리한다.

근거: `before_sources.zip`, `before.json`, `baseline-off.json`, `baseline-on.json`, `verify.json`, `applied.json`, `stage_tests.log`, `tests.log`, `config/tests/expiry_reference_r218_sha256.json`. 큰 재생 출력 캐시는 Git 제외 `build/r218_replay/`에 두고 해시를 결과에 기록했다.

WSL 일반 진입이 `Wsl/Service/E_UNEXPECTED`로 실패해 `/bin/true`에서도 재현했다. 시스템 WSL에서 PID2가 Ubuntu24.04/systemd인 것을 확인하고 기존 namespace 접근 도구로 같은 사용자/환경에서 재생을 마쳤다. 재부팅·서비스/전원 변경은 하지 않았다. 이 접근 장애를 제어기 실패와 합치지 않는다.

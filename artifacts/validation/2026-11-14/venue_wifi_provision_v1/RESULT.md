# Wi-Fi 등록·선택 전환 stage 결과

**Windows 모의14개 통과, PowerShell 파싱 오류0. 공통 미이관·실제 네트워크 무실행이다.** root의 기존 핫스팟 실연결 근거를 새 도구의 실제 전환 성공으로 합산하지 않는다.

변경은 신규 `scripts/jetson_wifi.ps1`, `jetson_wifi.py`, `jetson_wifi_remote.py`와 기존 `scripts/jetson_access.py`의 선택형 stdin 전달4줄이다. 기존 discovery/credential/호스트 키 RejectPolicy/SSH 구현을 재사용한다. 기존 전달 필드가 없는 호출은 원 sudo 암호 한 줄만 보내는 동작을 유지한다. 원 `jetson_access.ps1`, `discover_jetson.*`, `connect_jetson.ps1`은 변경하지 않았다.

기본 저장은 자동연결을 끈 새 root keyfile만 등록한다. `-Activate`에서 현재 연결 UUID를 읽고 복귀 timer를 먼저 예약·확인해야 전환을 예약한다. 새 망에서 별도 `-ConfirmTransition`이 명시되고 실제 목표 UUID가 활성일 때만 autoconnect=yes/priority100 적용 후 복귀 timer를 취소한다. 성공 등록 응답은 활성화 성공 증명이 아니며, 전환 예약 응답도 새 망 연결 성공 증명이 아니다.

검증14개는 저장만 기본 동작, keyfile 입력 escaping/SSID byte 경계, 잘못된 키 거절, 복귀→전환 순서, timer 실패 시 전환 금지, 전환 실패 시 복귀 유지, 확인 없음/옛 망에서 취소 금지, 목표 확인 후 자동연결100→취소 순서, 자동연결 변경 실패 시 복귀 유지, 만료/중복 전환, 비밀의 stdin 전용 전달, sudo 암호 소비/캐시 두 경우, 오류 출력의 비밀 억제, root0600/O_EXCL 파일 생성, 기존 SSH 키 정책/기존 호출 보존을 묶어 확인했다. 테스트의 암호는 합성 문자열이며 실제 credential 파일을 읽지 않았다.

`unittest_v1.log`와 `unittest_v2.log`를 보존했다. v2는 기존 stdin 필드 없음의 호환 assertion을 보완한 최종14 unique test이고 둘을28개로 합산하지 않는다. PowerShell AST로 `WifiPassword`가 `System.Security.SecureString`임을 확인했고 실제 PS 도구는 실행하지 않았다. 실제 네트워크 관련 OS 명령은 모두 fake로 대체했다. 모의 fixture 임시 폴더는 artifact 아래에서 생성·회수됐다. 실행 중인 자식/원격 PID 및 배정 자원은 없다.

주 담당의 통합 검토 항목은 새 stdin 전송4줄과 새 helper의 상태 전환 순서다. `candidate.patch`와 `changes.json`에 기존 원 SHA/최종 stage SHA를 기록한다. 기존 데이터는 덮어쓰지 않으며 NetworkManager에는 새 UUID 프로필만 만든다. PSK는 제어 명령·출력·임시 상태 JSON에 넣지 않는다.

**미확인:** 실제 NetworkManager 버전의 keyfile/SAE 처리, root 권한/실제 로그 비밀 비노출, systemd transient timer 실행·시간 만료 복귀, 새 AP 단말 격리/SSH 재발견, 부팅 자동연결. 이는 실망이 정해진 뒤 확인할 사항이며, 이번 stage 단계에서 Jetson/WSL/NIC에 접근하여 확대 검증하지 않았다. Windows 연결은 수동 UI 안내로 한정했다. root 전용 영구 NetworkManager keyfile에 PSK가 저장되는 것은 프로필 등록의 의도된 동작이며 비밀 미저장이라고 주장하지 않는다.

사용법과 공식 형식 근거는 [README.md](README.md)를 따른다. root가 source delta를 검토한 뒤 이관 여부를 결정할 수 있는 상태다.

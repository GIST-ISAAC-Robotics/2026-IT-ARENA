# 공통 Wi-Fi 저장 경로의 실제 NetworkManager 단1회 확인

2026-11-14, GPT. **저장 전용 시험과 자기 프로필 삭제까지 통과했다.** 실제 Wi-Fi 연결·전환 성공의 증거는 아니다.

기존 `jetson_access.ps1`의 mDNS/기존 credential/hostkey 경로로 `isaac-jetson` 연결을 확인했다. `wlP1p1s0`의 시작 활성 UUID는 `194f530b-f823-47f6-a6e0-aa927ea4ddad`였다. 존재하지 않는 공개 시험용 프로필 이름을 먼저 확인한 후, 공통 **`scripts/jetson_wifi.ps1`을 `-Activate` 없이 단 한 번 호출**했다. [공개 합성 fixture](fixture.json)는 leading/trailing spaces를 포함한 SSID와 공백·역슬래시를 포함한 합성 PSK다. 사용자 실제 암호를 사용·출력하지 않았다.

| 확인 | 실제 결과 |
|---|---|
| 공통 PowerShell 저장 경로 | `ok=true`, `status=stored_only`, `activation_scheduled=false` |
| 생성 UUID | `b6f7df01-b20a-46a2-b275-2ff49bdd836b` |
| keyfile 권한/소유자 | root:root, 0600 |
| NetworkManager 파싱 후 profile ID/SSID/PSK | 원 시험 값과 모두 일치. 출력에는 일치 boolean만 저장 |
| 자동연결 | false |
| 저장 직후 및 삭제 후 활성 UUID | 시작 UUID와 동일 |
| 기존 keyfile | 내용 hash 및 파일 목록 불변 |
| 회수 | 정확한 생성 UUID만 `nmcli connection delete uuid`로 삭제, exit0 |
| 삭제 확인 | 생성 UUID 없음, 정확한 keyfile 없음, 기존 파일 목록 복원 |

정확한 자기 파일은 `/etc/NetworkManager/system-connections/it-arena-venue-b6f7df01-b20a-46a2-b275-2ff49bdd836b.nmconnection`이었다. 삭제 전 UUID와 공개 시험 profile ID를 다시 대조했다. 다른 프로필을 수정하거나 지우지 않았다.

[등록 영수증](registration.json)·[확인/회수 결과](result.json)·[실행 harness](run_store.ps1)·[읽기/회수 helper](audit_remote.py)를 보존한다. `before.exit`, `registration.exit`, `result.exit`와 Windows 외부 명령은 모두0이다. 모든 명령은 동기 실행으로 종료됐고 백그라운드 프로세스/타이머를 만들지 않았다. helper PID6777을 포함한 동기 SSH 명령이 반환된 뒤 **Jetson 소유권을 root에 반환했다.**

활성 전환, 복귀 timer, 센서/포트/ROS/WSL, 전력/네트워크 장치 설정, 기존 프로필 수정은 수행하지 않았다. 신규 패키지·공통 코드 변경도 없다. 이번 범위는 실제 NetworkManager의 keyfile 로딩·escaping·비밀 권한·저장 전용 동작 확인까지다.

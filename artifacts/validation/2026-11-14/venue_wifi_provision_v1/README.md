# 대회장 Wi-Fi 등록·선택 전환 도구 stage

공통 미이관·주 담당 검토 대기 상태다. 실제 Jetson/NIC 전환은 수행하지 않았다. 기존 아이폰 핫스팟 연결 확인과는 별도다. 신규3파일과 `jetson_access.py`의 선택형 stdin 전달4줄이 [검토 대상](stage/scripts)이다.

## 사용 흐름

노트북과 Jetson을 먼저 기존 아이폰 핫스팟에 연결한다. 아래 예시는 저장소 루트 PowerShell에서 stage를 검토 후 실행할 때의 명령이다. 암호는 인수에 넣지 않고 가려진 입력창에서 입력한다. WPA2/WPA3 Personal만 지원하며 `-Security`를 실제 AP에 맞춘다.

```powershell
$tool = './artifacts/validation/2026-11-14/venue_wifi_provision_v1/stage/scripts/jetson_wifi.ps1'
$repo = (Get-Location).Path
& $tool -RepositoryRoot $repo -Ssid '대회장 SSID' -Security WPA3
```

기본 동작은 **새 프로필 저장만**이며 `autoconnect=false`다. 현재 연결/기존 핫스팟은 바꾸지 않는다. 응답의 `connection_uuid`를 보관한다. 매번 새 UUID로 등록하므로 기존 프로필을 다시 쓰려면 등록을 반복하지 않고 저장한 UUID를 선택한다.

```powershell
& $tool -RepositoryRoot $repo -ConnectionUuid '응답의-실제-UUID' -Activate
```

전환 직전의 실제 활성 연결 UUID로 **180초 복귀 타이머를 먼저 예약·확인**한 뒤3초 지연 전환을 예약한다. 필요하면 `-RollbackSeconds 300`처럼60–600초에서 정한다. 전환 ID가 비밀 없이 출력된다. 이제 노트북은 Windows Wi-Fi 메뉴에서 같은 대회장 SSID로 수동 연결한다. 이 도구는 Windows 프로필을 등록하거나 NIC를 전환하지 않는다.

새 망에서 노트북↔Jetson 통신을 확인한 뒤 아래 명령을 **명시적으로** 실행한다. 기존 mDNS 발견과 SSH 호스트 키·인증을 다시 통과해야 하며, Jetson의 활성 UUID도 목표와 일치해야 한다.

```powershell
& $tool -RepositoryRoot $repo -ConfirmTransition -TransitionId '출력된32자리전환ID'
```

확인 성공 시 대회장 프로필을 `autoconnect=yes`, 우선순위100으로 설정한 다음 복귀 타이머를 취소한다. 기존 핫스팟 우선순위50보다 높아 다음 연결 선택 때 우선 대상이 된다. 이미 연결된 다른 망에서 즉시 강제 이동한다는 뜻은 아니다. 등록/전환을 부팅 때마다 반복할 필요는 없다.

새 망에서 접속되지 않으면 확인 명령을 실행하지 않는다. 기존 핫스팟을 계속 켜 두고 복귀 시한 뒤 노트북도 핫스팟으로 돌아간다. 타이머/전환/확인 오류 때 도구는 복귀 타이머를 자동 취소하지 않는다. SSH 응답이 끊겨도 전환이 시작됐을 수 있으므로 같은 전환을 바로 중복 실행하지 않는다. 원격 일회성 timer는 재부팅을 넘겨 보장되지 않는다.

기본 Jetson interface는 기존 확인된 `wlP1p1s0`다. 다른 장비에서는 `-JetsonInterface`로 지정한다. `-InterfaceAlias`/`-ConnectAddress`는 기존 발견 도구 방식이며 명시 IP에서도 기존 호스트 키 검증을 유지한다. 설치 후 공통 `scripts/jetson_wifi.ps1`에서는 `RepositoryRoot`를 생략할 수 있다.

## 비밀과 권한

- 입력은 `Read-Host -AsSecureString` 또는 SecureString 파라미터다. 변환된 평문은 프로세스 메모리/자식 stdin/암호화된 SSH stdin으로만 전달한다. 관리되는 문자열의 RAM 완전 소거까지 보장하지 않는다.
- PSK를 argv, 환경변수, 로컬 파일, 로그에 쓰지 않는다. 전송용 SFTP 파일도 없다. sudo command에는 비밀 없는 helper 코드만 들어간다.
- 실제 저장 위치는 Jetson의 root:root0600 NetworkManager keyfile이다. 같은 root 디렉터리의0600 임시 사본을 완성 후 이름 변경하며 임시 파일은 정상/실패 경로에서 정리한다. 실패한 `nmcli load` 뒤 영구 keyfile은 남을 수 있다.
- 복귀 상태는 `/run/it-arena-wifi`0700 아래0600 JSON이며 UUID/interface/전환 ID/기한만 담는다. PSK는 없다. 성공 확인 뒤 상태 파일을 지우고, 실패/시간 만료의 비밀 없는 상태는 다음 점검을 위해 남긴다.
- helper는 nmcli stdout/stderr와 예외 원문을 출력하지 않는다. 장치 로그에서 PSK가 남지 않았다는 실제 감사는 이번 모의 범위에 포함되지 않는다.

## 검증과 한계

[RESULT.md](RESULT.md)에14개 Windows 모의 검사와 PowerShell 정적 파싱 결과를 정리했다. 실제 대회장 SSID가 정해지지 않았으므로 AP 인증/격리/혼잡도, 실제 NetworkManager keyfile 파싱·권한·timer 동작은 이번에 실행하지 않았다. 802.1X/EAP·웹 로그인·open AP·Windows 자동 등록은 범위 밖이다. 현재 key 입력은8–63 ASCII 문자 또는 WPA2의64 hex PSK다.

keyfile의 root 전용 권한과 SSID byte-list 표현은 [NetworkManager 공식 문서](https://www.networkmanager.dev/docs/api/latest/nm-settings-keyfile.html), transient timer 방식은 [systemd-run 공식 문서](https://www.freedesktop.org/software/systemd/man/247/systemd-run.html)를 따랐다. 실제 배포 환경의 동작 확인은 mock 결과와 구분한다.

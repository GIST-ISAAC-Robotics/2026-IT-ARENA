# Jetson 접속 정보와 비공개 자격 증명

2026-11-14 갱신. 이전 환경·시험 이력은 [A3 기록](../simulation/JETSON_REPLAY_2026_09_28.md), 현재 실행 결과는 [실물 입력 보고서](../reports/RUNTIME_SENSOR_INTEGRATION_2026_11_12.md)에서 관리한다.

- SSH 대상: **`q@isaac-jetson.local`**, 포트 `22`. 11/13 사용자 요청으로 `jetson-orin`에서 변경했다. 사용자 계정·SSH 호스트 키·IP 할당 방식은 유지했다.
- 같은 Wi-Fi에서 이름으로 접속한다. 이번 조회의 `192.168.0.134`는 DHCP 주소이며 고정 주소로 가정하지 않는다.
- 원문: 사용자 바탕화면의 `jetson_orin_nano_jp621_headless_runbook.md`. 원문은 변경하지 않았다.
- 분리 저장: `%LOCALAPPDATA%\IT-Arena\private\jetson-access.clixml`. Windows DPAPI와 디렉터리 접근 권한을 적용했고 저장 후 재읽기를 검증했다. 같은 Windows 사용자/컴퓨터에서 사용하며, Git에 넣지 않는다.
- SSH 호스트 키: `%USERPROFILE%\.ssh\known_hosts`. 키가 달라지거나 등록되어 있지 않으면 중단한다. 자동 신뢰로 바꾸지 않는다.

## 사용

이 노트북에서는 바탕화면의 **`ISAAC Jetson 접속.cmd`**를 실행한다. 같은 Wi-Fi에 직접 이름을 질의해 현재 IPv4를 찾고, 기존 장비의 SSH 키가 일치할 때만 터미널을 연다. 비밀번호는 SSH의 입력 창에 평소처럼 입력한다. 이 문서와 바로가기에는 비밀번호를 넣지 않는다.

터미널에서는 `./scripts/connect_jetson.ps1`도 같은 동작이다. `-ProbeOnly`는 장비 발견·키 확인만 하고 로그인하지 않는다. Windows SSH 별칭 `ssh isaac`와 `ssh q@isaac-jetson.local`도 준비되어 있으나, 운영체제 이름 조회가 잘못되는 환경에서는 바탕화면/스크립트 방식을 우선한다. NoMachine 접속을 새로 만들 때도 새 이름 또는 확인된 현재 IP를 사용한다.

자동 시험은 저장소 루트에서 다음과 같이 실행한다.

```powershell
./scripts/jetson_access.ps1 -Command 'hostname'
```

[PowerShell 도구](../../scripts/jetson_access.ps1)는 `.local` 대상에 숫자 주소를 지정하지 않으면 [mDNS 발견·기존 키 확인](../../scripts/discover_jetson.py)을 먼저 수행한다. 확인 뒤 암호화된 자격 증명을 읽어 [SSH 도구](../../scripts/jetson_access.py)의 표준 입력으로만 전달한다. 비밀번호를 명령행 인수·환경변수·로그에 기록하지 않는다. `run`·`put`·`get` 작업과 선택형 로그를 지원하며, 파일 전송은 기존 목적지를 덮어쓰지 않는다. 관리자 실행은 명시적인 `-Sudo`에서만 사용한다. 발견은 접속 준비 단계이며 주행 제어 루프에서 호출하지 않는다.

네트워크 인터페이스 이름이 `Wi-Fi`가 아니면 `-InterfaceAlias '<이름>'`을 지정한다. 공유기가 mDNS를 막되 장비 간 IP 통신은 허용한다면, 관리자가 알려준 현재 IPv4로 `./scripts/connect_jetson.ps1 -ConnectAddress '<IPv4>'`를 사용한다. 수동 주소에서도 기존 장비 키 검사는 유지한다. 아무 주소에나 고정 IP를 배정하거나 키 경고를 무시하지 않는다.

현재 Windows Python은 `C:/Python314/python.exe`, SSH 의존성은 Git 제외 경로 `build/jetson_tools`의 Paramiko다. 해당 디렉터리가 없으면 이 환경에서 다음 명령으로 다시 설치한다.

```powershell
C:/Python314/python.exe -m pip install --target build/jetson_tools paramiko
```

이 도구는 실행 승인을 대체하지 않는다. 원격 설치·설정 변경과 자료 전송은 작업 범위와 승인에 따라 별도로 수행한다. `-Timeout`으로 연결 도구가 종료된 경우 원격 작업까지 멈췄다고 가정하지 말고 해당 프로세스를 확인한다.

## 이름 조회만 실패할 때

### 대회장 핫스팟으로 먼저 연결하기 — 11/14 확인

노트북과 Jetson에 사용자 아이폰 핫스팟을 저장하고 자동 연결을 켰다. Jetson 프로필은 `ISAAC-iPhone-Hotspot`, 자동 연결 우선순위는50이며 로그인 사용자 제한이 없다. 암호는 Windows Wi-Fi 저장소와 Jetson의 root 전용 NetworkManager 설정(0600)에만 남겼고, 전송용 임시 암호 파일은 삭제했다. 공개 저장소에는 암호를 기록하지 않는다.

1. 아이폰에서 **개인용 핫스팟 → 다른 사람의 연결 허용**을 켜고, 연결될 때까지 그 설정 화면을 열어둔다.
2. Jetson을 부팅하고 노트북을 같은 핫스팟에 연결한다. 저장된 이름·암호가 같고 검색 가능하면 자동 연결 대상이다. 이미 다른 Wi-Fi에 연결돼 있으면 우선순위가 높다는 이유만으로 갈아타지는 않는다.
3. 바탕화면 `ISAAC Jetson 접속.cmd` 또는 `scripts/connect_jetson.ps1`로 접속한다. 네트워크 변경 직후 이름 조회가 실패하면 잠시 뒤 다시 시도한다. 이번 시험에서도 변경 직후의 실패와 이후 mDNS·SSH 성공을 모두 확인했다.
4. 처음 쓰는 대회장 Wi-Fi는 이름·암호를 한 번 전달해야 한다. 다른 팀 장치와의 격리 정책에 따라 같은 AP에 연결돼도 노트북↔Jetson 접속이 막힐 수 있으므로, 통신 확인 전까지 핫스팟을 복구 경로로 유지한다. 원격 전환은 기존 연결 복귀 타이머를 걸고 수행한다.

11/14에는 기존 연결을 해제한 뒤 **NetworkManager가 핫스팟을 자동 선택한 로그**, DHCP 주소 취득, 같은 핫스팟의 노트북에서 기존 SSH 키 확인·로그인·명령 실행, mDNS 발견을 확인했다. 시험 주소 `172.20.10.4`는 고정 주소가 아니다. 재부팅 시험이나 실제 대회장 AP 시험은 하지 않았다. 최종적으로 양쪽 모두 기존 `issac`로 복귀했고 임시 복귀 타이머도 종료했다. [시험 근거](../../artifacts/validation/2026-11-14/wifi_bootstrap_v1/README.md).

### 대회장 Wi-Fi를 노트북에서 등록·전환

공통 도구 `scripts/jetson_wifi.ps1`를 연결했다. **등록 기본값은 저장만**이며 현재 Wi-Fi를 끊지 않는다. 새 대회장 네트워크를 알게 되면 노트북과 Jetson을 먼저 핫스팟에 연결한 뒤 저장소 루트 PowerShell에서 실행한다. 암호는 가려진 입력창에 입력하며 명령문이나 공개 기록에 붙이지 않는다. 아래 WPA2는 예시이며 AP에 맞춰 WPA3를 선택한다.

```powershell
./scripts/jetson_wifi.ps1 -Ssid '대회장 SSID' -Security WPA2
```

응답의 `connection_uuid`를 사용해 전환한다. 기존 연결로180초 뒤 복귀하는 타이머가 먼저 예약된다. 노트북은 Windows Wi-Fi 메뉴에서 새 망으로 연결한다.

```powershell
./scripts/jetson_wifi.ps1 -ConnectionUuid '응답의 UUID' -Activate
```

새 망에서 Jetson 접속이 가능한 상태에서 다음 명령으로 확인한다. 기존 SSH 호스트 키와 실제 활성 프로필을 확인한 경우에만 복귀 타이머를 취소하고 자동연결 우선순위100을 적용한다.

```powershell
./scripts/jetson_wifi.ps1 -ConfirmTransition -TransitionId '출력된 전환 ID'
```

새 망 접속이 안 되면 확인 명령을 실행하지 않고 핫스팟을 계속 켜 둔다. 복귀 시한 뒤 노트북도 핫스팟으로 돌아간다. SSH 응답이 끊겨도 전환은 진행됐을 수 있으므로 등록/전환을 바로 반복하지 않는다. 필요하면 전환 시 `-RollbackSeconds 300`으로 여유를 늘린다. Windows Wi-Fi는 이 도구가 자동 전환하지 않는다.

WPA2/WPA3 Personal만 지원한다. [공통 이관·모의14검사](../../artifacts/validation/2026-11-14/venue_wifi_provision_v1/integration_v1/README.md)와 [실제 Jetson 저장 전용 1회](../../artifacts/validation/2026-11-14/venue_wifi_provision_v1/live_store_v1/RESULT.md)를 마쳤다. 실제 저장 시험에서는 합성 SSID/암호의 NetworkManager 읽기 값·0600 권한·자동연결 OFF와 기존 연결 불변을 확인하고 시험 프로필만 삭제했다. **실제 대회장 AP 전환과 새 도구의 timer 복귀는 아직 미검증**이며, 저장 성공이나 기존 핫스팟 연결 성공과 구분한다.

### 일반 Wi-Fi 이름 조회

11/13에는 공유기 DNS에 직접 질의한 `jetson-orin.local`과 Windows 캐시가 외부 주소를 반환했지만, 같은 Wi-Fi의 mDNS 응답은 실제 Jetson의 IPv4를 반환했다. 그 주소에서 기존 SSH 키 일치와 인증 성공을 확인했다. 따라서 이번 장애는 **이름 조회 경로의 잘못된 응답**까지 확인했으며, 공유기 캐시·설정·상위 DNS 중 어디서 생겼는지는 미확정이다. 당시 Avahi 서비스는 정상이고 같은 이름 충돌의 직접 증거는 없었다. 젯슨 전원/센서 장애로 취급하지 않는다.

`.local`은 로컬 링크의 mDNS 이름이다([RFC 6762](https://www.rfc-editor.org/rfc/rfc6762.html)). 새 도구는 공유기의 일반 DNS 응답을 사용하지 않고 해당 Wi-Fi의 mDNS 후보를 얻은 뒤 기존 SSH 키로 확인한다. 같은 이름의 다른 장비, 미등록 키, 복수의 확인된 주소는 자동 선택하지 않는다. 공유기에서 클라이언트 격리가 켜져 있으면 이름을 바꿔도 접속은 되지 않으므로 대회장에서는 **노트북↔Jetson 통신 허용 여부**가 필요하다. 대회장 AP 실증은 아직 하지 않았다.

이름 변경 전 `/etc/hostname`, `/etc/hosts`, Avahi 설정은 Jetson의 `/var/backups/it-arena-network/20261113T011829Z/`에 보존했다. 노트북의 비공개 프로필·기존 SSH 설정/키 목록도 `%LOCALAPPDATA%\IT-Arena\private\network-name-20261113T012329Z\`에 보존했다. Avahi만 다시 시작했으며 네트워크 재연결·재부팅·센서 서비스 변경은 하지 않았다. 새 이름 mDNS·키 확인과 자동 도구의 실제 `hostname` 응답 `isaac-jetson`까지 확인했다.

### 이전 현상 기록

9/29 기본 Windows OpenSSH는 `jetson-orin.local`을 Wi-Fi의 IPv6 링크 로컬 주소로 해석하고 기존 호스트 키도 일치했지만, Python/Paramiko 도구는 같은 이름에 `getaddrinfo failed`를 반환한 경우가 있었다. 이 경우를 젯슨 전원 장애로 단정하지 않는다. 두 조회 경로가 다른 근본 원인은 아직 확정하지 않았다.

기본 SSH의 `-v -o BatchMode=yes -o StrictHostKeyChecking=yes` 출력 등으로 **현재 주소와 기존 키 일치**를 확인한 경우에만 `jetson_access.ps1 -ConnectAddress '<확인된 숫자 IP>' -Command 'hostname'`을 사용할 수 있다. `BatchMode=yes`는 비밀번호 입력을 금지하므로 키 확인 뒤 `Permission denied`로 끝날 수 있으며, 이 진단 명령만으로 계정 비밀번호가 틀렸다고 판단하지 않는다. 이 선택은 TCP 연결 주소만 바꾸며 키 검증 이름은 저장된 장비명(현재 `isaac-jetson.local`)을 사용한다. 미등록 키 자동 허용이나 기존 키 교체는 하지 않는다. IPv6 링크 로컬 주소의 `%인터페이스번호`는 PC별 값이므로 다른 컴퓨터에 그대로 복사하지 않는다. 임시 주소를 영구 기본값으로 저장하지 않는다.

## 가상 화면·원격 GUI의 비용

바탕화면 원본 runbook §26–28과 실제 `/etc/X11/xorg.conf`를 대조했다. 현재는 NVIDIA `CustomEDID`와 `ConnectedMonitor=DFP-0`로 가상 1920×1080 모니터를 유지하고, GDM/Xorg/GNOME 및 NoMachine 9.8.2가 실행된다. 단순 파일만 놓인 상태가 아니라 GUI 프로세스가 존재한다. 원본의 계정/암호 부분은 이 저장소에 복사하지 않았다.

11/13 10:27 KST, 원격 화면의 TCP4000 연결이 없는 유휴 상태에서 10.06초 동안 관련 11프로세스의 CPU 시간과 PSS를 읽었다. 합계는 **한 CPU 코어의 0.994%, PSS 545.5MiB**다. 이어진 `tegrastats` 1초 간격 5표본에서 GPU 활성률 0%, RAM 2,692/7,620MiB, 시스템 `MemAvailable` 약4.71GiB였다. PSS는 공유 메모리를 분담한 수치이며, 서비스를 끌 때 정확히 그만큼 반환된다는 뜻은 아니다. [NVIDIA 지표 정의](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/AT/JetsonLinuxDevelopmentTools/TegrastatsUtility.html).

현재 유휴 GUI가 제어 계산 지연의 주원인이라는 근거는 없다. 원격 화면 전송·영상 재생 중 비용이나 GUI ON/OFF의 실제 판단 지연 비교는 하지 않았다. 지금은 설정을 유지하고 주행 중 불필요한 NoMachine 화면 전송을 피한다. 추후 메모리가 부족하면 센서 취득·GUI 의존성을 확인한 뒤 GUI 중지 상태를 별도 비교한다. NoMachine은 X 서버가 없을 때 자체 가상 화면을 만들 수도 있으므로, `display-manager`만 중지했다고 GUI 비용 전체가 없어지는 것으로 보지 않는다([공식 설명](https://kb.nomachine.com/AR03P00973)). 이번 점검에서는 화면/서비스·전력 설정을 바꾸지 않았다.

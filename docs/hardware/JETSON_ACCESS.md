# Jetson 접속 정보와 비공개 자격 증명

2026-09-28 기준. 실제 환경과 시험 결과는 [A3 기록](../simulation/JETSON_REPLAY_2026_09_28.md)에서 관리한다.

- SSH 대상: `q@jetson-orin.local`, 포트 `22`.
- 같은 Wi-Fi에서 이름으로 접속한다. 이번 조회의 `192.168.0.134`는 DHCP 주소이며 고정 주소로 가정하지 않는다.
- 원문: 사용자 바탕화면의 `jetson_orin_nano_jp621_headless_runbook.md`. 원문은 변경하지 않았다.
- 분리 저장: `%LOCALAPPDATA%\IT-Arena\private\jetson-access.clixml`. Windows DPAPI와 디렉터리 접근 권한을 적용했고 저장 후 재읽기를 검증했다. 같은 Windows 사용자/컴퓨터에서 사용하며, Git에 넣지 않는다.
- SSH 호스트 키: `%USERPROFILE%\.ssh\known_hosts`. 키가 달라지거나 등록되어 있지 않으면 중단한다. 자동 신뢰로 바꾸지 않는다.

## 사용

일반 터미널에서는 `ssh q@jetson-orin.local`로 접속할 수 있다. 이 문서에는 비밀번호를 적지 않는다.

자동 시험은 저장소 루트에서 다음과 같이 실행한다.

```powershell
./scripts/jetson_access.ps1 -Command 'hostname'
```

[PowerShell 도구](../../scripts/jetson_access.ps1)는 암호화된 자격 증명을 읽어 [SSH 도구](../../scripts/jetson_access.py)의 표준 입력으로만 전달한다. 비밀번호를 명령행 인수·환경변수·로그에 기록하지 않는다. `run`·`put`·`get` 작업과 선택형 로그를 지원하며, 파일 전송은 기존 목적지를 덮어쓰지 않는다. 관리자 실행은 명시적인 `-Sudo`에서만 사용한다.

현재 Windows Python은 `C:/Python314/python.exe`, SSH 의존성은 Git 제외 경로 `build/jetson_tools`의 Paramiko다. 해당 디렉터리가 없으면 이 환경에서 다음 명령으로 다시 설치한다.

```powershell
C:/Python314/python.exe -m pip install --target build/jetson_tools paramiko
```

이 도구는 실행 승인을 대체하지 않는다. 원격 설치·설정 변경과 자료 전송은 작업 범위와 승인에 따라 별도로 수행한다. `-Timeout`으로 연결 도구가 종료된 경우 원격 작업까지 멈췄다고 가정하지 말고 해당 프로세스를 확인한다.

## 이름 조회만 실패할 때

9/29 기본 Windows OpenSSH는 `jetson-orin.local`을 Wi-Fi의 IPv6 링크 로컬 주소로 해석하고 기존 호스트 키도 일치했지만, Python/Paramiko 도구는 같은 이름에 `getaddrinfo failed`를 반환한 경우가 있었다. 이 경우를 젯슨 전원 장애로 단정하지 않는다. 두 조회 경로가 다른 근본 원인은 아직 확정하지 않았다.

기본 SSH의 `-v -o BatchMode=yes -o StrictHostKeyChecking=yes` 출력 등으로 **현재 주소와 기존 키 일치**를 확인한 경우에만 `jetson_access.ps1 -ConnectAddress '<확인된 숫자 IP>' -Command 'hostname'`을 사용할 수 있다. `BatchMode=yes`는 비밀번호 입력을 금지하므로 키 확인 뒤 `Permission denied`로 끝날 수 있으며, 이 진단 명령만으로 계정 비밀번호가 틀렸다고 판단하지 않는다. 이 선택은 TCP 연결 주소만 바꾸며 키 검증 이름은 계속 저장된 `jetson-orin.local`이다. 미등록 키 자동 허용이나 기존 키 교체는 하지 않는다. IPv6 링크 로컬 주소의 `%인터페이스번호`는 PC별 값이므로 다른 컴퓨터에 그대로 복사하지 않는다. 임시 주소를 영구 기본값으로 저장하지 않는다.

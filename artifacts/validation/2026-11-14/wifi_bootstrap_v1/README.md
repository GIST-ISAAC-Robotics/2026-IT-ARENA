# 대회장 접속 준비: 아이폰 핫스팟

2026-11-14 KST. 사용자 승인 범위에서 노트북·Jetson Wi-Fi 프로필을 등록하고 실제 연결을 확인했다. 주행 시험·센서 접근·재부팅은 하지 않았다.

| 항목 | 확인 결과 |
|---|---|
| Windows | 현재 사용자 Wi-Fi 프로필 저장, 자동 연결, 실제 WPA3-Personal 연결 성공 |
| Jetson | `ISAAC-iPhone-Hotspot`, autoconnect=yes, priority=50, 사용자 로그인 제한 없음 |
| 자동 선택 | 기존 `issac` 연결 해제 후 02:12:45 및 02:15:18에 NetworkManager `auto-activating connection` 확인 |
| 주소 취득 | 02:15:19에 DHCP `172.20.10.4/28`; 재사용을 보장하는 고정 IP 아님 |
| 장치 간 통신 | 핫스팟의 노트북에서 기존 키로 SSH 인증 후 `hostname`과 실제 연결 상태 읽기 성공 |
| 이름 발견 | `isaac-jetson.local`의 mDNS 후보1개·기존 키 일치, `verified_unique_device` |
| 보안 | Jetson 시스템 keyfile root:root 0600, 전송용 로컬/원격 암호 파일 삭제. 공개 기록에 암호 없음 |
| 종료 | 양쪽 `issac` 복귀, 기존 주소에서 SSH와 mDNS 재확인, 시험용 복귀 타이머/로컬 helper 종료 |

처음에는 Windows 검색 목록에서 핫스팟이 사라져 재연결 요청이 거절됐다. 사용자에게 핫스팟 설정 화면을 열어달라고 요청한 뒤 검색을 갱신하고 연결했다. 설정 화면이 닫힌 것이 유일한 원인이었다고 단정하지 않는다. 첫 전환 직후의 mDNS 실패도 보존하며, 이후 같은 핫스팟에서 이름 발견과 SSH가 성공했으므로 이를 핫스팟의 상시 단말 격리로 해석하지 않는다.

이번 자동 연결 검사는 `nmcli connection down`으로 기존 연결을 해제하고 NetworkManager가 저장 프로필을 선택하는 방식이다. **Jetson 부팅 자체, 장시간 잠긴 아이폰의 재발견, 실제 대회장 AP/혼잡도는 미시험**이다. 처음 사용하는 대회장 네트워크는 SSID·인증 정보 등록이 필요하다.

선별 원기록: [Jetson 연결·자동 선택·파일 권한](jetson_connection_receipt.txt). [운영 방법](../../../../docs/hardware/JETSON_ACCESS.md).

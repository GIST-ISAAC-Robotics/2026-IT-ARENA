# UNO Q·임시 센서 설정 오프라인 통합 결과

## 완료 범위

2026-11-14 사용자 요청에 따라 GPT가 현장 보유 UNO Q용 native MCU 펌웨어, Jetson UART/USB 서보 호스트, 대기 중 IMU 임시 보정과 차량 자체 영역 마스크를 준비했다. 모든 장치 OFF 상태의 PC 작업이며 실제 포트 열기·SSH·flash·GPIO·모터/서보 출력·ROS graph·Gazebo 실행은 없다. 다른 PC 주행 성능 개선의 중단 범위는 유지했다.

- [선택 통합 검사](pytest.xml): **136개 통과**, 10.20초. MCU portable C++ ASan/UBSan 및 크기 최적화, host/observation/profile, IMU 추정, 자체 마스크, 배포 검사를 포함한다.
- [최종 추출 배포본 검사](package_v3/package_pytest.xml): **132개 통과**, 6.67초. MCU C++ 5개 묶음은 앞 검사에서 검증했으므로 반복하지 않았고, 패키징 연쇄 의존성 회귀 1개를 추가했다. 두 검사 수를 합산하지 않는다.
- [파일 검증 전](package_v3/package_inventory.json)과 [검증 후](package_v3/package_inventory_after.json): 목록 381개 및 SHA256 일치. 테스트는 추출한 배포본의 소스만 사용했다.
- [호스트 진단](package_v3/package_host_dryrun.json), [5%·5초 단발 인수](package_v3/package_bench_dryrun.json), [native flash 계획](package_v3/package_flash_dryrun.json): 모두 dry-run 성공, 장치 접근 없음. flash 백업 디렉터리도 생성되지 않았다.
- [임시 설정 재생성](package_v3/package_quick_setup.json)은 배포한 YAML과 byte 단위로 동일하다. front/back 20cm × left/right 15cm에 사방3cm, 최종26×21cm. `base_link` 중심 오프셋 x=0.00365m. 기존 후방/CAD 각도 마스크를 해당 overlay에서 끈다.
- 관련 문서5개의 로컬 링크31개 존재를 확인했고, 변경 추적 파일의 `git diff --check`를 통과했다.

## 산출물

최종 로컬 ZIP은 `build/uno_q_deployment_20261114_v3/uno_q_deployment.zip`이다. Git 제외 build 산출물이며 최신 dirty source를 파일별로 식별한다.

```text
ZIP SHA256
5aaa70c5521db4ba6cef5d4f9ba046161a11384e282c5ebbf0dc46b3f66a70a0

unconfigured zephyr.bin (94492 bytes)
362999b69a80d60b43e9be8367ee87722f3326e8cbc81a9bd58b7f28b1b8d854

bench zephyr.bin (94508 bytes)
6cf7b9ddf65a466c687c74e11cef553d9d42eca14cefb0fe9d282afcc0b6e7e5
```

두 firmware manifest의 source46개·artifact14개를 현재 원본과 대조했다. 자동 ARM은 없으며 installed 모터 PI/방향 프로필은 임의 생성하지 않았다. [설치 절차](../../../../docs/hardware/UNO_Q_DEPLOYMENT_2026_11_14.md)와 [설계 결정](../../../../docs/decisions/0030-uno-q-replacement-controller.md)을 따른다.

## 실패·수정 보존

첫 통합 실행 준비에서는 ROS setup을 `set -u` 뒤 source하여 unset 변수 오류가 발생했다. setup 뒤 nounset을 켜도록 실행 스크립트를 고쳤다. quick setup 검사 fixture의 필수 상태값 누락과 표준 LaserScan 각도 범위를 벗어난 fixture도 수정했다. 판정 조건을 완화하지 않았다.

v1은 ZIP 설치와 Windows 호스트/bench/flash dry-run까지 통과했다. v2의 [추출 검사](package_pytest.xml)는130통과·1실패다. `test_uno_q_host_profile` 함수 안에서 import한 `test_motor_feedback_runtime` 보조 파일이 빠진 패키징 실패였다. 수동으로 한 파일만 넣지 않고 같은 디렉터리 로컬 import를 재귀 추적하도록 생성기를 수정했다. 해당 반례 회귀를 추가한 v3에서132개를 통과했다. v2 결과를 성공으로 덮어쓰지 않았다.

## 실물에서 남은 확인

실제 MCU 업로드는 UNO Q 자체 Linux에서 원 Arduino MCU loader를 백업하고 교체하는 절차다. Linux eMMC OS는 대상이 아니다. 현재 OpenOCD 설치·J12 pinmux·외부 하네스·냉간부팅·PWM0 타이밍·USB 서보 단절·동시 부하는 미검증이다. M01 입력24V 미만 정격 문제도 남는다.

엔코더 방향·모터 DIR·PI와 주행 상한은 실측 설치 프로필이 필요하다. IMU는 잠정 추정이며 수평 전후축/부호는 충분한 가감속·좌우 회전 관측이 있어야 확정 후보가 된다. MCU 방향/PI를 자동으로 바꾸지 않는다. 박스 내부 실제 외부 물체도 제외될 수 있다. 이번 PC 검사와 cross-build는 실차 안전·전체 코스 완주 증명이 아니다.

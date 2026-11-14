# UNO Q 펌웨어와 Jetson 호스트 이관 — 2026-11-14

> 경기 종료 후 분류: **현장 미사용 대체안/준비 이력**. 대회용 최종 기준은 [ESP32 소스 보존본](../releases/FINAL_ESP32_2026_11_14.md)이다. 아래 내용은 당시 오프라인 준비 기록이며 실차 적용 결과가 아니다.

## 범위와 현재 코드

사용자가 현장 보유 Arduino UNO Q로 고장 의심 ESP32를 대체하도록 요청했다. 이번 작업은 모든 장치가 꺼진 상태의 **펌웨어·호스트 구현, PC 검증, 설치 묶음 준비**다. Jetson/UNO Q 접속·GPIO 변경·flash·모터/서보 출력은 실행하지 않는다. 물리 STOP 스위치는 최신 사용자 지시에 따라 포함하지 않는다.

기존 `firmware_runtime_drive`의 portable 구동 계약·PI 제어와 Jetson `run_runtime_actuation_host.py`가 이식 기준이다. 기존 ESP32 바이너리를 UNO Q에 쓰지 않는다. 새 MCU는 STM32U585용 native Zephyr이며 Jetson과 **USART1 1,000,000 baud, 8N1**로 통신한다. 내부 Linux↔MCU Router의 115200 baud 링크는 운영 경로에 사용하지 않는다.

최신 작업 트리의 자율주행 수정과 MCU 교체 작업을 구별한다. r541 실측 영점 적용 및 r557 정책 수정 이후 r560은 사용자 중단으로 부분 주행만 남았다. 이전 `selected_v3`의 미채택 후보를 설치 대상으로 사용하지 않는다. 후속 사용자가 현장 임시 자동 보정을 추가 요청하여 아래 센서 처리 범위만 구현한다. 별도 작업의 PC 전체 주행 중단은 유지한다. 묶음에는 해당 기능을 포함한 현재 자율주행 소스도 들어가지만 **전체 코스에서 검증된 새 릴리스는 아니다.**

## 하드웨어 연결과 보존할 실측

[전체 회로 SVG](schematics/uno_q_all_v1.svg), [핀별 표](schematics/uno_q_pin_table_v1.md), [설계 JSON](schematics/uno_q_v1.json)을 기준으로 한다.

| 기능 | 선정 연결 |
|---|---|
| Jetson↔MCU | J12 8 TX→D0/PB7, D1/PB6→J12 10 RX, J12 14↔GND①. UART 내부 pull 없음 |
| 엔코더 | CF A1→A2/PA6 TIM3_CH1, A2→A3/PA7 TIM3_CH2. TIM3 x4 counter |
| 모터 | D9/PB8 TIM4_CH3 PWM 20 kHz, D4/PA12 DIR. MD20A 신호 GND↔GND④ |
| IMU | SPI2 D13/SCK, D11/MOSI, D12/MISO, D10/CS; D2/INT1. 기존 FIFO 표본 계약 유지 |
| 서보 | Jetson USB-C→데이터 C-to-C→Adapter(A) USB-C. 점퍼 두 개 B, H2 UART 미연결 |
| UNO Q 전원 | M01 CH6/CN7 12 V→VIN, GND②. 운영 중 UNO Q USB 미연결 |
| STOP | 물리 입력/스위치 없음. 소프트웨어 STOP·명령 만료는 유지 |

조향 실측은 중앙 **2811**, 왼쪽 **3031**, 오른쪽 **2550 counts**다. 전동 목표는 **2600–2980 counts**를 사용한다. 중앙 등가각 상한 **0.28475436568260193 rad**는 실측 영점에 CAD 각도 관계를 결합한 값이며, 바퀴각 실측은 아니다. [보정 JSON](../../config/hardware/steering_zero.assembled_2026_11_14.json)을 포함한다. ARM 시 중앙으로 이동하지 않고 **현재 위치 읽기→그 위치 유지 목표 ACK→토크 ON ACK**를 거친다.

기존 ESP32의 손 회전 +690 counts는 사용자가 **후진**으로 정정했다. 새 TIM3 계수 부호와 분해능은 재확인 대상이다. 모터 방향·PI 게인은 실측값이 없어 임의의 정상값으로 설정하지 않는다.

전기적 미해결 조건도 남는다. M01 제조사 입력 정격은 24–36 V이므로 6S 배터리 전압이 24 V 아래일 때의 운영은 정격 밖이다. UNO Q VIN에 완충 6S를 직접 넣지 않는다. 기존 발열 ESP32는 전원·USB·외부선을 분리한다. 이번 빌드와 PC 시험은 외부 하네스 건전성·전원 강하·역급전·부팅 핀 상태·실물 PWM0 증거가 아니다.

## MCU와 USB 서보 사이의 계약

기존 IA JSON 명령/ACK/offer의 의미와 `/drive`→독립 보호층→`/drive/safe` 경계를 유지한다. 새 `ESV1`은 같은 IA framing의 별도 49-byte payload다.

```text
<4s8sQQQIBq (little endian)
magic, boot, generation, lease, status_seq, observation_age_us, ready, applied_drive_seq
```

MCU가 발행한 현재 lease에 상태를 묶고, 유효기한을 `min(lease.expires, lease.issued + 100000 us - observation_age_us)`로 계산한다. 수신 시각만 기준으로 유효기간을 연장하지 않는다. 서보 관측이 오래되거나 응답이 실패하면 MCU가 출력을 차단한다. MCU에서 수락한 DRIVE 순번과 서보 목표 ACK 순번이 일치해야 모터 출력을 허용한다. 실제 ARM 성공 때만 기존 절대 만료 시각을 유지해 다음 generation으로 준비 상태를 넘기며 STOP/CLEAR/DISARM/FAULT는 무효화한다.

## 설치 묶음 검증과 복사

배포 ZIP은 빌드에 사용한 소스 해시, `zephyr.elf`/`zephyr.bin` 해시, 빌드 도구 버전, 호스트 소스, 실측 보정, pyserial 3.5 wheel을 함께 보존한다. `git HEAD`만으로 최신 소스를 대표하지 않으며 작업 트리의 실제 파일 해시를 사용한다. 빌드 뒤 firmware/portable 소스가 바뀌면 묶음 생성을 거부한다.

| 묶음 안 위치 | 사용 범위 |
|---|---|
| `firmware/unconfigured/` | 센서·통신 진단. runtime ARM 거절, PWM0 |
| `firmware/bench/` | 명시적인 모터 단독 단발. 최대15%·5초, 만료/엔코더 조건에서 조기 종료 가능 |
| `scripts/uno_q_host.py` | Jetson 진단·선택 ROS 연결·벤치·설치 프로필 기반 runtime |
| `config/hardware/uno_q_quick_setup_2026_11_14.yaml` | 자동 IMU 임시 보정과 box-only 자체 반환 제외를 합친 준비된 overlay |

두 이미지는 부팅 미무장이다. 설치용 PI/방향 프로필을 확정한 runtime 이미지는 포함하지 않는다. 해당 출력 경로 구현과 합성 프로필 검사는 완료했으나 실제 값이 없어 임의로 채우지 않았다. 향후 build script에 검증한 `uno_q_profile_v1`을 제공하면 installed 이미지가 생성된다. 호스트는 그 프로필 지문과 자체 설치 설정이 맞을 때만 ARM을 허용한다.

전송 후 압축을 새 폴더에 풀고 실행한다. 아래 명령은 검증과 파일 복사만 한다. 서비스 등록·자동 ARM·포트 열기·flash는 하지 않는다.

```bash
export PYTHONDONTWRITEBYTECODE=1
python3 scripts/deploy_uno_q_bundle.py .
python3 scripts/deploy_uno_q_bundle.py . --install --destination "$HOME/it-arena-uno-q-20261114"
cd "$HOME/it-arena-uno-q-20261114"
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --no-index --find-links wheelhouse -r requirements-jetson.txt
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
```

현재 Jetson에 설치된 ROS 환경을 source한 뒤 사용한다. Python/ROS ABI는 Jetson의 기존 환경을 유지한다. `rclpy`, `ackermann_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`는 기존 ROS 설치가 제공해야 하며 wheel에 담긴 것으로 가장하지 않는다. `venv`가 없으면 Jetson OS에 맞는 패키지 준비가 필요하다. 장치가 꺼져 있으므로 현재 설치 여부는 조회하지 않았다.

### 읽기 전용 사전 조회

```bash
.venv/bin/python scripts/check_uno_q_jetson.py --power-status --output jetson_preflight.json
```

장치 노드 등록·권한, ROS import 가능 여부, L4T와 전력 모드 조회 결과를 기록한다. UART를 열거나 pinmux·전력 모드를 변경하지 않는다. `/dev/ttyTHS*`가 있다는 사실만으로 J12 대응을 확정하지 않는다. J12 8/10 UART 기능과 console 점유를 실제 Jetson에서 확인한 뒤 MCU 포트를 선택한다. 서보는 `/dev/serial/by-id/…`의 실제 장치를 식별한다.

### MCU 업로드 준비와 첫 호스트 연결

이번 native Zephyr 이미지는 `0x08000000`에서 시작하여 **UNO Q의 기존 Arduino MCU loader를 교체**한다. Linux eMMC/OS는 건드리지 않는다. [MCU flash·백업·복원 문서](../../src/arena_vehicle_interface/firmware_uno_q/FLASH.md)에 실제 실행 전 조건과 복원 명령을 적었다. 백업2회 해시·같은 MCU UID·바이너리 벡터를 확인하며 option bytes/unlock/mass erase는 사용하지 않는다.

```bash
# 어느 PC에서든 가능한 기본 계획 검사. 장치/프로세스 호출 없음.
python3 firmware/bench/flash_native.py --firmware-dir firmware/bench --backup-dir "$HOME/uno-q-original-backup"
python3 scripts/uno_q_host.py --dry-run
```

실제 flash는 **UNO Q 자체 Linux**에서 진행한다. 같은 묶음의 `firmware/bench` 또는 `firmware/unconfigured`를 복사한 뒤 FLASH.md 절차를 따른다. 단순히 Jetson에서 위 경로를 실행한다고 UNO Q가 업로드되는 것은 아니다. 현재 UNO Q에 설치된 OpenOCD와 핀 설정·냉간부팅은 아직 확인하지 않았다.

향후 기기가 준비되고 MCU/USB 장치 경로를 확인한 뒤 사용할 Jetson 명령은 다음과 같다. MCU는 하나의 호스트만 연다.

```bash
# 원시 관측. 서보를 연결하지 않았다면 --servo-port 항목을 생략한다.
.venv/bin/python scripts/uno_q_host.py --diagnostics --mcu-port "$MCU_UART" \
  --servo-port "$SERVO_USB" --seconds 30 --output "$NEW_LOG_DIRECTORY"
# 자동 보정용 ROS 관측까지 발행하려면 위 명령에 --ros를 추가한다.

# 출력 없이 벤치 인수만 검증: 5%·5초 요청, DIR=0을 전진이라고 가정하지 않는다.
.venv/bin/python scripts/uno_q_host.py --dry-run --bench \
  --bench-duty-permille 50 --bench-direction 0 --bench-duration-ms 5000
```

실제 벤치는 `--dry-run`을 제거하고 확인한 `--mcu-port`, 새 `--output`을 명시할 때만 실행한다. 서보 전원을 분리한 모터 단독 모드이며 서보 토크를 켜지 않는다. 출력은 한 번뿐이고 KEEPALIVE가 최초 종료시각을 연장하지 않는다. 5초는 최대 요청시간이며 엔코더 진행이 없으면 먼저 끈다. [호스트 상세 계약과 토픽](UNO_Q_HOST_PREPARATION_2026_11_14.md).

## 사용자 요청 임시 자동 보정

대기 상태에서 실제 정지 후보가 **1.5초 이상** 유지되면 gyro 3축 바이어스와 중력 기반 위쪽 축을 자동 추정한다. 명령0·신선한 엔코더 또는 원시 count 불변·가속도 크기/분산·자이로 분산을 함께 확인한다. 중력을 가속도 바이어스로 빼지 않는다. 적용 시 운동 이력을 비워 이전 좌표/부호와 섞지 않으며, 주행 중 바이어스를 계속 바꾸지 않는다.

평면 yaw는 위쪽 축에 gyro를 투영하므로 전후축 추정 완료를 기다리지 않는다. 이후 가감속의 수평 가속도와 엔코더 변화, **실제 서보 count에 근거한 조향각**과 좌/우 회전의 일관성으로 전후축·엔코더 부호 후보를 수집한다. 증거가 부족하면 미확정 상태를 표시한다. 이 결과를 MCU의 PI/모터 DIR까지 교정했다고 표시하지 않는다. [자동 IMU 보정 구현](IMU_QUICK_CALIBRATION_2026_11_14.md).

사용자 확인한 자체 영역은 차량 중심 기준 **앞뒤20cm × 좌우15cm + 사방3cm**, 즉 **26×21cm**다. 현재 기하 중심은 `base_link`의 x=0.00365m이므로 x=[−0.12635,0.13365]m, y=[−0.105,0.105]m를 사용한다. 영역 안 반환은 자체 반사로 제외하고, 원시 토픽과 밖의 반환을 보존한다. 가림 뒤를 자유 공간으로 채우지 않는다. 박스 안 실제 외부 물체도 제외될 수 있는 임시 근사다. [마스크 구현과 검증](../sensors/QUICK_SELF_MASK_2026_11_14.md).

아래 명령은 설정 파일만 만들며 노드·장치를 실행하지 않는다.

바로 사용할 수 있는 `config/hardware/uno_q_quick_setup_2026_11_14.yaml`도 묶음에 포함했다. 재생성 또는 기존 설정과 합칠 때만 다음 도구가 필요하다.

```bash
.venv/bin/python scripts/prepare_uno_q_quick_setup.py --output quick_setup.yaml
# 현재 ROS parameter YAML을 하나로 합칠 때:
.venv/bin/python scripts/prepare_uno_q_quick_setup.py --base-params current_runtime.yaml --output quick_runtime.yaml
```

기존 운용 노드의 ROS 인자에 `--params-file quick_setup.yaml`을 추가하고, **계획기와 독립 LiDAR 보호층 양쪽에 같은 파일을 적용**한다. 생성기는 `/hardware_runtime/imu`를 지정하고 마스크 JSON을 본문 문자열로 넣는다. 기존 후방/CAD 각도 마스크를 끄고 CAD 구조각 기반 rear-wall witness도 끄므로 box 밖 반환을 후단에서 다시 삭제하지 않는다. 기존 주행 정책·속도·PI 값은 유지하며, 현재 운용의 센서 위치/시간 설정은 함께 전달해야 한다. 자체 영역/보정 프로필을 켠 것이 MCU ARM 권한은 아니다.

포함된 `src/arena_autonomy`와 `src/arena_vehicle_interface`를 Jetson의 기존 ROS 환경에서 별도 build/install 폴더로 빌드한다. x86 native `.so`는 묶음에 넣지 않는다. 이전 설치 폴더를 덮어쓰지 않는다.

```bash
colcon --log-base quick-build-log build --base-paths src/arena_autonomy src/arena_vehicle_interface \
  --build-base quick-build --install-base quick-install \
  --packages-select arena_vehicle_interface arena_autonomy
source quick-install/setup.bash
```

## 후속 실물 순서

1. 장치가 다시 사용 가능해지면 배선·전원 극성을 회로와 대조하고, 구동 전원이 분리된 상태에서 MCU 업로드와 재부팅을 확인한다.
2. 출력 비활성 진단으로 UART CRC/boot/sequence, IMU FIFO, 엔코더 손 회전, USB 서보 ID/설정/현재 위치를 확인한다.
3. 명시적인 단발 벤치 시험에서 PWM0 복귀·명령 만료와 실제 회전을 확인한다. 모터 방향·엔코더 부호/분해능을 기록한다. 자동 출력 배증은 하지 않는다.
4. 실측 모터 프로필과 PI 설정을 준비하고, 실제 설치 프로필을 검증해 주행 출력을 켠다. `/drive/safe` 구독은 기존 보호층 이후에만 둔다.
5. 호스트 종료·UART 분리·USB 서보 응답 상실·전체 동시 부하에서 실제 출력 중단을 계측한다. 전원 재투입 뒤 자동 ARM/중앙 이동이 없는지도 확인한다.

## 공식 자료

- [Arduino UNO Q 매뉴얼](https://docs.arduino.cc/tutorials/uno-q/user-manual): Linux MPU와 STM32 MCU를 구분한다.
- [Arduino Core on Zephyr 소스](https://github.com/arduino/ArduinoCore-zephyr): Arduino 업로드·실행 경로의 기준이다.
- [Arduino Linux OS flash 안내](https://docs.arduino.cc/software/app-lab/configure/flash/): 전체 Linux OS 복구용이다. 이번 MCU 앱 업로드와 혼동하지 않는다.
- [STM32U585 공식 데이터시트](https://www.st.com/resource/en/datasheet/stm32u585ai.pdf): 핀 AF와 전기적 조건의 기준이다.

## 오프라인 완료 근거

- 두 native ARM ELF/bin cross-build 완료. unconfigured bin **94,492 bytes**, bench bin **94,508 bytes**. 각 manifest의 소스46개·산출물14개와 SHA256을 대조했다. ARM ELF32 little endian/hard-float이며 flash vector는0x08000000이다.
- MCU의 ASan/UBSan·최적화 C++ 및 Python5개 검사 묶음: 원 IA 계약, ESV 만료/재생/세대, servo ACK 불일치, encoder wrap, BNV 제한/정상 종료, 기본 dry-run 무장치 경계. 정상 bench 종료와1ms guard의 경계 분류 문제를 수정한 최종 바이너리다.
- 호스트 관련220개, 자동 IMU 보정 관련141개, 자체 영역 마스크 관련 중복 제외271개 회귀를 각각 통과했다. 서로 겹치는 검사를 합산하지 않는다.
- 최종 선택 통합 검사 **136개 통과**: [통합 XML](../../artifacts/validation/2026-11-14/uno_q_integration_v1/pytest.xml). 실제 ROS 메시지 클래스·가짜 노드/통신·plan/current 자식 IPC·native C++를 사용했으며 ROS graph/장치/주행 실행은 없다.
- 패키지 해시/추출 후 dry-run 결과는 배포 출력 폴더의 `bundle_result.json`과 통합 검증 기록에 남긴다. 파일 설치와 장치 업로드는 별도다.

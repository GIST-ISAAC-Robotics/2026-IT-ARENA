# 조립 직후 조향 서보 수동 3점 측정과 미동

2026-11-14. ST3215-HS와 ESP32가 차량에 연결되면 구동 모터 전원을 분리하고 앞바퀴를 띄운 상태에서 사용한다. 기존 무부하 시험에서 쓴 [`servo_probe_v1`](../../src/arena_vehicle_interface/firmware_servo_probe/README.md)을 재사용한다. Jetson의 `~/it_arena_steering_setup_20261114`에 도구와 동일 해시의 ESP32-S3 앱이 준비되어 있다. ROS와 자율주행은 필요 없다.

**11/14 약 12:47 설치 확인:** 사용자가 서보·구동 모터 전원을 모두 분리한 상태에서 기존 ESP32-S3와 파티션을 대조했다. 직전 앱 1 MiB를 보존하고 `0x10000`에 `servo_probe_v1`만 설치했다. 독립 `verify-flash`의 digest 일치와 `INFO`의 `armed=false`, `motor_pwm=0`, `motor_dir=0`, 포트 해제를 확인했다. 첫 별도 검증은 esptool 5.1.0에서 `verify-flash --no-progress`가 거절됐고, 해당 옵션 없이 쓰기 재시도 없이 검증을 마쳤다. 장치 원문은 Jetson 비공개 실행 폴더에 남겼다. 서보 전원은 이 단계에서 인가하지 않았으므로 토크나 장착각을 읽지 않았다.

현재 장치에서 앱 변경이 없었다면 다음은 **서보 전원만 연결한 뒤 2절의 `release`와 수동 3점 측정**이다. 복구가 필요할 때는 Jetson `~/it_arena_steering_setup_20261114/runs-preflight.YJqe2b/previous-app.bin`이 이번 직전 백업이다. 같은 바이트의 사본을 노트북의 Git 제외 `build/mounted_servo_setup_20261114_v1/`에도 확보했다.

같은 작업 중 Jetson은 `MAXN_SUPER`(모드 2), CPU 6개 online이었다. 짧은 관측에서 모듈 `VDD_IN` 약 4.8–5.1 W, 접합 온도 약 38°C, OC1/2/3 카운터 각각 0을 확인했다. Jetson 재부팅이나 전력 설정 변경은 하지 않았다. 이는 해당 부하의 확인이며 J01 19 V 단자 전압 실측이나 최대 부하 시험이 아니다. [NVIDIA 전력·이벤트 기준](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html#overcurrent-event-status)

## 1. ESP32를 연결하고 진단 앱 준비

1. 구동 모터 전원을 **물리적으로 분리**하고, 펌웨어 확인·교체 중에는 **서보 전원도 끈다**. ESP32의 USB-UART 포트만 Jetson에 연결한다. `ls -l /dev/serial/by-id/`로 연결 전후를 비교해 실제 USB-UART를 찾는다. 해당 디렉터리가 없으면 새로 나타나는 `/dev/ttyUSB*`를 확인한다. 포트를 다른 서비스나 시험이 사용 중이면 먼저 종료한다. 번호를 추측하여 선택하지 않는다.
2. Jetson의 대화형 셸에서 아래 경로와 변수를 설정한다. 현재 Jetson 계정은 `dialout` 소속이 아니므로 아래 장치 명령에만 `sudo`를 사용한다.

```bash
cd ~/it_arena_steering_setup_20261114
umask 077
TOOLS="$HOME/it_arena_imu_20261111/tools_venv/bin"
PORT='/dev/serial/by-id/실제_USB_UART_이름'
RUN="$PWD/runs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN"
```

3. 이미 해당 진단 앱이 설치되어 있으면 백업/업로드가 필요 없다. 그 외에는 **이전에 시험한 동일 ESP32-S3 보드와 파티션 구성이 맞을 때** 아래를 실행한다. 파티션 해시는 11/12 실물 백업의 `0x8000` 섹터 기준이다. 해시 불일치나 읽기 오류가 나면 `write-flash` 전에 중단된다. 1 MiB 앱 원본을 먼저 저장하며, 부트로더·파티션·NVS는 쓰지 않는다. [esptool 문서](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/esptool/basic-commands.html)

```bash
(
  set -eu
  test -c "$PORT"
  test ! -e "$RUN/previous-app.bin"
  printf '%s  %s\n' 64647128b46fe6a9f145b1c951a4c91f9dc2b798beeb6b4beb4ece8c33aeb645 arena_servo_probe.bin | sha256sum -c -
  sudo "$TOOLS/esptool" --chip esp32s3 --port "$PORT" -b 460800 read-flash 0x8000 0x1000 "$RUN/partition-sector.bin"
  printf '%s  %s\n' b9118aa3840f7c042a4dea8dae339d07a6e69f3716102174cc87b18046276295 "$RUN/partition-sector.bin" | sha256sum -c -
  sudo "$TOOLS/esptool" --chip esp32s3 --port "$PORT" -b 460800 read-flash 0x10000 0x100000 "$RUN/previous-app.bin"
  test "$(stat -c %s "$RUN/previous-app.bin")" -eq 1048576
  (cd "$RUN"; sha256sum previous-app.bin > previous-app.sha256)
  sudo "$TOOLS/esptool" --chip esp32s3 --port "$PORT" -b 460800 write-flash 0x10000 arena_servo_probe.bin
)
```

4. 전원 OFF 상태에서 [차량 배선](ESP32_GPIO_WIRING_2026_11_11.md)을 확인한다. Adapter A 점퍼 두 개 모두 **A**, ESP32 GPIO17→TX, GPIO18↔RX, 공통 GND, 서보 D/V/G는 케이블·보드 표기 대조, Adapter A 전원은 **M01 CH5(CN6)**다. 서보에만 전원을 인가하고 구동 모터 전원은 분리한 상태를 유지한다. [Adapter A 결선 기준](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example)

## 2. 토크 해제 → 손으로 직진·왼쪽 최대·오른쪽 최대 기록

```bash
servo() {
  local label="$1"; shift
  sudo "$TOOLS/python" scripts/mounted_servo_setup.py --port "$PORT" --log "$RUN/$label" "$@"
}
servo release release
```

`torque_after: 0`을 확인한 뒤 다음 한 줄을 실행한다. 프로그램이 세 자세를 순서대로 안내하며, 그동안 서보는 위치 읽기만 한다. 서보 전원은 위치 피드백을 위해 켜 두고 토크는 계속 끈 상태다.

`release`에서 ESP32의 `disarmed`만 돌아오고 서보 응답이 없다면 토크 OFF는 미확인이다. 수동 측정은 보류하고 어댑터 전원 LED, 점퍼 두 개 모두 A, GPIO17→TX·GPIO18↔RX·공통 GND 및 서보 D/V/G를 확인한다. 점퍼나 커넥터 조정 전에는 서보 전원을 분리한다.

```bash
servo manual measure --assembly-id arena-car-v1 --motor-power-disconnected --wheels-raised
```

1. **직진:** 자나 각도계로 차체 전후 방향에 맞추고, 토우가 있다면 좌우 대칭을 본다. 바퀴를 그 자세로 가볍게 유지하며 Enter를 누르고 기록 메시지가 나올 때까지 기다린다.
2. **왼쪽 최대:** 운전자 기준 왼쪽으로 손으로 천천히 돌린다. 링크가 닿거나 저항이 뚜렷해지면 더 밀지 말고, 무리 없이 유지되는 마지막 자세에서 Enter를 누른다. 오른쪽도 같은 방식으로 기록한다. 서보의 기어를 억지로 역구동하거나 끝단에 힘을 주지 않는다.
3. **실제 바퀴각(선택):** 각 끝점에서 각도계로 재려면 자세를 유지한 채 각 바퀴의 직진 기준 대비 회전량을 잰다. 안내가 나오면 `28.5 22`처럼 **왼쪽 바퀴, 오른쪽 바퀴** 순서로 각도의 크기를 입력한다. 프로그램이 좌회전에는 `+`, 우회전에는 `−`를 붙인다. 각도계가 없으면 Enter로 생략한다. 서보 count에서 환산한 **서보 축 각도**는 저장되지만 바퀴각은 빈값으로 남는다.

`$RUN/manual/summary.json`에 세 자세의 서보 count, 좌우 끝의 영점 대비 서보 축 각도, 입력한 바퀴각이 남는다. `$RUN/manual/straight.json`은 기존 교정 도구가 읽는 영점 프로필이다. 수치 경계 0/4095를 넘은 경우나 두 끝이 영점의 같은 쪽에 나온 경우는 `issues`에 표시하고, 범위와 방향 후보를 내지 않는다. 측정 뒤 앞바퀴를 손으로 직진에 되돌린다.

**2048에 강제로 이동하거나 서보의 내부 오프셋을 변경할 필요는 없다.** 링크가 중립에서도 비틀리거나 걸리면 전원을 끄고 혼이나 링크를 조정하고 새 로그 디렉터리에서 다시 측정한다. 영점만 필요할 때는 기존 `zero` 명령(`--wheels-straight`, `--assembly-id`, `--evidence`, `--output`), 현재 위치만 읽을 때는 `servo read read`를 사용한다.

## 3. 노트북 조향 제어 창

손으로 링크를 움직이기 어려우면 억지로 돌리지 않고, Windows 작업 폴더에서 [조향 창 실행기](../../scripts/open_steering_panel.ps1)를 실행한다.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\open_steering_panel.ps1
```

1. 모터 전원은 분리하고 앞바퀴를 띄운다. 서보 전원을 연결한 뒤 **서보 연결 확인**을 눌러 토크 OFF와 위치를 읽는다. 바퀴에 손이나 공구가 닿지 않는 상태에서 확인란을 선택한다.
2. **한 번에 이동**에서 12/24/36/48 counts(서보축 약 1.05/2.11/3.16/4.22°)를 고른 뒤 **− / +**를 눌러 직진을 맞추고 **중앙 저장**을 누른다. 버튼당 목표 하나만 전달한다. 12 counts 미도달 뒤 눈에 보이는 간섭이 없다면 먼저 24 counts 한 번을 관찰한다. 토크 제한은 최대 30%를 유지하며, 목표와 실제 위치·토크 OFF 확인을 창에 표시한다. 서보 count 부호는 실제 좌우를 보고 판단한다.
3. **양쪽 끝 탐색 허용**을 선택하면 왼쪽, 오른쪽을 차례로 저장할 수 있다. 끝점에 가까워지면 12 counts를 선택한다. 손/공구로 링크를 억지로 움직이지 않고 간섭 직전에서 멈춘다. 끝점 바퀴각 두 칸은 각도계로 잰 값이 있을 때만 입력한다. 탐색 상한은 중앙 ±275 counts이며, CAD 근처에서 탐색을 제한하는 값이다. 실물 끝단이나 운용 허용 범위가 아니다.
4. 기록은 화면에 표시된 Jetson `panel-run-*` 폴더의 `summary.json`과 `straight.json`에 저장된다. 마치면 **토크 해제**를 눌러 OFF를 확인한다. 창은 SSH 연결 하나만 사용하며, 닫으면 연결과 포트를 반환한다.

최초 현장 확인에서 어댑터 전원 미연결로 ID 1이 8개 속도 모두 무응답이었다. 전원 연결 뒤 서보 토크 레지스터 OFF와 위치 2612 counts를 읽었다. 이후 최초 미동 요청에서 현재 목표 2611을 쓰자 토크가 켜졌으며, 위치는 2611을 유지했다. 이동 목표 2599는 보내지 않았고 토크 OFF를 재확인했다. 호스트 도구를 수정해 토크 제한과 현재 위치 확인 뒤 요청 목표만 쓰도록 했다. 수정 뒤 원격 재요청은 서보 전원을 다시 빼는 동안 설정 읽기에서 종료되어 이동 명령은 없었다. 창과 Jetson SSH 연결, 모의 명령·세 지점 저장을 확인했으며 수정된 실제 이동은 아직 미확인이다.

**장착 상태 첫 실측:** `+12 counts` 요청 세 번(2611→2623, 2612→2624 두 번)에서 목표 쓰기·목표 조회·토크 ON은 응답했으나, 1초씩의 위치 피드백 60표본에서 각 시작점 기준 최대 0–1 count만 변했다. 서보 보고 전압 12.1 V·상태 0, 정착 시 목표 오차는 성공 기준 5 counts를 넘었다. 세 차례 모두 토크 OFF를 읽었으며 중립·끝점은 저장하지 않았다. 장착 상태의 간섭·마찰과 작은 위치 차에서 시작하기에 부족한 구동량을 아직 구분하지 못했다. 저항이 느껴지면 목표나 토크를 키우기 전에 전원을 끄고 링크/혼의 접촉을 확인한다. 필요할 때는 링크를 분리하여 어디에서 움직임이 막히는지 구분한다. 기존 ±114 counts 무부하 결과는 이 작은 장착 시험의 대체 근거가 아니다.

### 중앙 나사 조정 후 수동 3점

사용자는 중앙 나사가 과하게 조여 있었으며 이를 조정하자 손으로 조향할 수 있게 됐다고 보고했다. 이 상태에서 앞바퀴를 손으로 맞추고, Jetson에서 매 지점의 토크 OFF와 안정된 서보 위치를 확인했다.

| 차량 기준 자세 | 서보 위치 | 중앙 대비 서보 축 각도 |
|---|---:|---:|
| 중앙 직진 | 2811 counts | 0° |
| 왼쪽 최대 | 3031 counts | +19.336° |
| 오른쪽 최대 | 2550 counts | −22.939° |

수치 wrap은 없다. count 증가가 차량 왼쪽이며 `count_direction_per_cad_positive=-1`이다. 양쪽 **바퀴 자체의 각도는 미측정**이다. 이 끝점은 손으로 관찰한 값이며 운용 상한으로 적용하지 않았다. Jetson 내부의 `manual-capture.MMQBmq/`에 원자료를, 노트북의 Git 제외 `build/mounted_servo_setup_20261114_v1/manual-capture.MMQBmq-verified/`에 `straight.json`과 `summary.json`의 사본을 보존한다.

첫 전동 시험은 오른쪽 2550에서 중앙 방향으로 36 counts를 요청해 실제 2580까지 30 counts 움직였다. 목표 오차 6 counts로 기존 5 counts 정밀 기준은 미달했고 토크 OFF는 확인했다. 별도 기능 시험은 시작 전에 정착·해제 뒤 오차 10 counts 이내, 요청 방향으로 이동, 상태/전류 이상 없음, 최대 48 counts 단발, 양쪽 끝 25 counts 이상의 목표 여유를 조건으로 정했다. 이 10 counts 기준으로 기존의 미도달을 소급 통과 처리하지 않는다.

### 전동 저속 왕복 확인

오른쪽에서 안쪽으로 추가 1회 확인한 뒤, 앞바퀴를 띄우고 구동 모터 전원을 분리한 상태에서 1회 왕복했다. 토크 제한 30% 이하와 기존 저속 설정을 유지했다. 목표는 최대 48 counts 간격의 단발로 주었고, 각 스텝의 토크 OFF readback 뒤 다음으로 진행했다. 중간 목표 2780 → 왼쪽 안쪽 2980 → 오른쪽 안쪽 2600 → 중앙 2811 순서였다.

- **23/23** 요청 방향으로 이동했고, 정착 오차 최대 **6 counts**, 토크 해제 뒤 최대 **7 counts**였다. 10 counts 기능 기준 통과, 별도 5 counts 기준은 **15/23**이다.
- 왕복 중 실제 **2606–2976 counts**였고 마지막은 **2807 counts**(중앙보다 −4, 서보축 약 −0.35°)였다. 마지막 토크 OFF와 포트 반환을 확인했다. 서보 보고 전압 12.0–12.1 V, 상태 오류 0, 온도 30–31°C였다.
- Jetson `manual-capture.MMQBmq/powered-sweep-full/`에 UART·피드백·집계가 있다. `powered-sweep-summary.json`을 노트북의 위 검증 사본 폴더에 보존했다. 수동 끝점까지의 전동 동작, 접지 상태 하중, 실제 바퀴각, 운영 한계는 별도다.

### 터미널에서 한 번에 한 방향씩 움직여 관찰

아래 명령은 **각각 한 줄씩** 실행한다. 손과 도구를 링크에서 빼고, 첫 이동 방향·걸림·떨림과 토크 해제를 확인한 뒤 다음 명령을 실행한다. 차체 앞에서 보더라도 방향은 운전자 기준으로 기록한다.

```bash
step() {
  local label="$1"; shift
  servo "$label" jog --profile "$RUN/manual/straight.json" \
    --motor-power-disconnected --wheels-raised "$@"
}
step plus-01 --counts=12
step back-01 --toward-zero
step minus-01 --counts=-12
step back-02 --toward-zero
```

- `+12`/`−12`는 **서보 축 약 ±1.05°**이며, 차량의 오른쪽/왼쪽은 실제 바퀴를 보고 결정한다. `--toward-zero`는 한 번에 최대 12 counts만 중립 쪽으로 이동한다. 버튼처럼 매번 새 로그 이름을 사용한다. 토크 해제 뒤 링크 하중으로 되밀릴 수 있으므로 `position_before_release`와 `position_after_release`를 구분한다.
- 초기 미동 범위는 중립 ±56 counts(서보축 약 ±4.92°)이며, 한 번에 최대 12 counts다. 수치 끝 0/4095를 넘어가는 명령은 거부한다. **제한에 걸리면 덮어쓰거나 wrap하지 말고 혼의 장착 위치를 확인**한다. 초기 미동 허용 범위는 실제 기계적 끝단이나 경기 조향 범위가 아니다.
- [CAD의 전체 설계 범위](STEERING_FUSION_2026_11_13.md)까지 쓰려면 중립 양쪽에 약 266–269 counts의 수치 여유도 필요하다. 예전 벤치값 3879는 높은 쪽에 216 counts만 남는다. 실제 조립 중립이 끝에 가깝고 더 큰 범위가 필요하면 서보 전원을 분리한 상태에서 혼/링크 결합을 다시 잡고 재측정한다. 조립한 링크를 건 채로 서보를 2048로 보내지 않는다.
- 토크 OFF와 현재 위치를 확인하고 SRAM 토크 설정을 최대 30%로 제한한 뒤, 요청한 작은 목표만 한 번 쓴다. 목표 쓰기 자체가 토크를 켤 수 있다. 토크 한도는 실제 축 토크 실측값이 아니다. 이동마다 토크 OFF를 명령하고 서보 레지스터를 확인한다. 통신이 끊기거나 토크 OFF 확인이 없으면 서보 전원을 끈다. 떨림·기구 걸림·위치 미도달이면 더 큰 값으로 재시도하지 않는다. 이 앱의 1초 무요청 만료는 UART 버스가 살아 있을 때만 토크 해제 요청을 전달할 수 있다. [ST 레지스터 기준](https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation)

## 4. 마무리와 운영 설정 인계

`servo done release`로 토크 OFF를 다시 읽고 서보 전원을 끈다. 임시 앱을 올렸다면 다음으로 **이번 설치 직전** 앱을 복구하고 Flash 내용을 검증한다. `BACKUP`에는 `previous-app.bin`이 실제 저장된 디렉터리를 지정한다. 11/12 백업으로 대체하지 않는다. 토크 상한 변경은 SRAM이므로 서보 전원 재인가 시 저장 설정으로 돌아온다.

```bash
(
  set -eu
  BACKUP="$RUN" # 다른 실행에서 저장했다면 그 디렉터리로 바꾼다.
  (cd "$BACKUP"; sha256sum -c previous-app.sha256)
  sudo "$TOOLS/esptool" --chip esp32s3 --port "$PORT" -b 460800 write-flash 0x10000 "$BACKUP/previous-app.bin"
  sudo "$TOOLS/esptool" --chip esp32s3 --port "$PORT" -b 460800 verify-flash 0x10000 "$BACKUP/previous-app.bin"
)
```

`straight.json`은 **중립만** 저장한다. 최대점 측정 결과는 `summary.json`에서 확인한다. `+count`가 실제 오른쪽이면 CAD 방향값 `+1`, 왼쪽이면 `−1`이다. 손으로 확인한 끝점에서 기계 여유를 남겨 운영 범위를 따로 정한 뒤, 실제 바퀴각과 링크 여유를 확인하고 [현장 절차의 `confirm-mount` 및 운영 설정](ASSEMBLED_CAR_BOX_CHECK_2026_11_14.md#서보라이다엔코더)으로 넘긴다. 측정값을 경기용 상한으로 자동 적용하지 않는다.

호스트 기능 검증: Jetson ARM 가상 시리얼에서 수동 3점의 위치·바퀴각 저장과 ARM/서보 쓰기 0, 토크가 켜져 있으면 측정 중단, 기존 미동과 상태 이상 시 토크 해제 readback을 검사했다. 실제 장착 각도, 부하 및 서보 토크 해제는 서보 전원 연결 뒤 확인한다.

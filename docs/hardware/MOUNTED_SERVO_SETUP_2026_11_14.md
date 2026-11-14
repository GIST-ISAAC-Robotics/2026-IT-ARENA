# 조립 직후 조향 서보 수동 3점 측정과 미동

2026-11-14. ST3215-HS와 ESP32가 차량에 연결되면 구동 모터 전원을 분리하고 앞바퀴를 띄운 상태에서 사용한다. 기존 무부하 시험에서 쓴 [`servo_probe_v1`](../../src/arena_vehicle_interface/firmware_servo_probe/README.md)을 재사용한다. Jetson의 `~/it_arena_steering_setup_20261114`에 도구와 동일 해시의 ESP32-S3 앱이 준비되어 있다. ROS와 자율주행은 필요 없다.

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
  sha256sum "$RUN/previous-app.bin" > "$RUN/previous-app.sha256"
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

```bash
servo manual measure --assembly-id arena-car-v1 --motor-power-disconnected --wheels-raised
```

1. **직진:** 자나 각도계로 차체 전후 방향에 맞추고, 토우가 있다면 좌우 대칭을 본다. 바퀴를 그 자세로 가볍게 유지하며 Enter를 누르고 기록 메시지가 나올 때까지 기다린다.
2. **왼쪽 최대:** 운전자 기준 왼쪽으로 손으로 천천히 돌린다. 링크가 닿거나 저항이 뚜렷해지면 더 밀지 말고, 무리 없이 유지되는 마지막 자세에서 Enter를 누른다. 오른쪽도 같은 방식으로 기록한다. 서보의 기어를 억지로 역구동하거나 끝단에 힘을 주지 않는다.
3. **실제 바퀴각(선택):** 각 끝점에서 각도계로 재려면 자세를 유지한 채 각 바퀴의 직진 기준 대비 회전량을 잰다. 안내가 나오면 `28.5 22`처럼 **왼쪽 바퀴, 오른쪽 바퀴** 순서로 각도의 크기를 입력한다. 프로그램이 좌회전에는 `+`, 우회전에는 `−`를 붙인다. 각도계가 없으면 Enter로 생략한다. 서보 count에서 환산한 **서보 축 각도**는 저장되지만 바퀴각은 빈값으로 남는다.

`$RUN/manual/summary.json`에 세 자세의 서보 count, 좌우 끝의 영점 대비 서보 축 각도, 입력한 바퀴각이 남는다. `$RUN/manual/straight.json`은 기존 교정 도구가 읽는 영점 프로필이다. 수치 경계 0/4095를 넘은 경우나 두 끝이 영점의 같은 쪽에 나온 경우는 `issues`에 표시하고, 범위와 방향 후보를 내지 않는다. 측정 뒤 앞바퀴를 손으로 직진에 되돌린다.

**2048에 강제로 이동하거나 서보의 내부 오프셋을 변경할 필요는 없다.** 링크가 중립에서도 비틀리거나 걸리면 전원을 끄고 혼이나 링크를 조정하고 새 로그 디렉터리에서 다시 측정한다. 영점만 필요할 때는 기존 `zero` 명령(`--wheels-straight`, `--assembly-id`, `--evidence`, `--output`), 현재 위치만 읽을 때는 `servo read read`를 사용한다.

## 3. 한 번에 한 방향씩 움직여 관찰

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
- 현재 위치를 먼저 목표로 설정하고 SRAM 토크 설정을 최대 30%로 제한한 뒤 저속 이동한다. 이는 실제 축 토크 실측값이 아니다. 이동마다 토크 OFF를 명령하고 서보 레지스터를 확인한다. 통신이 끊기거나 토크 OFF 확인이 없으면 서보 전원을 끈다. 떨림·기구 걸림·위치 미도달이면 더 큰 값으로 재시도하지 않는다. 이 앱의 1초 무요청 만료는 UART 버스가 살아 있을 때만 토크 해제 요청을 전달할 수 있다. [ST 레지스터 기준](https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation)

## 4. 마무리와 운영 설정 인계

`servo done release`로 토크 OFF를 다시 읽고 서보 전원을 끈다. 임시 앱을 올렸다면 다음으로 **이번 실행 직전** 앱을 복구하고 Flash 내용을 검증한다. 11/12 백업으로 대체하지 않는다. 토크 상한 변경은 SRAM이므로 서보 전원 재인가 시 저장 설정으로 돌아온다.

```bash
(
  set -eu
  sha256sum -c "$RUN/previous-app.sha256"
  sudo "$TOOLS/esptool" --chip esp32s3 --port "$PORT" -b 460800 write-flash 0x10000 "$RUN/previous-app.bin"
  sudo "$TOOLS/esptool" --chip esp32s3 --port "$PORT" -b 460800 verify-flash 0x10000 "$RUN/previous-app.bin"
)
```

`straight.json`은 **중립만** 저장한다. 최대점 측정 결과는 `summary.json`에서 확인한다. `+count`가 실제 오른쪽이면 CAD 방향값 `+1`, 왼쪽이면 `−1`이다. 손으로 확인한 끝점에서 기계 여유를 남겨 운영 범위를 따로 정한 뒤, 실제 바퀴각과 링크 여유를 확인하고 [현장 절차의 `confirm-mount` 및 운영 설정](ASSEMBLED_CAR_BOX_CHECK_2026_11_14.md#서보라이다엔코더)으로 넘긴다. 측정값을 경기용 상한으로 자동 적용하지 않는다.

현재 검증: Jetson ARM 가상 시리얼에서 수동 3점의 위치·바퀴각 저장과 ARM/서보 쓰기 0, 토크가 켜져 있으면 측정 중단, 기존 미동과 상태 이상 시 토크 해제 readback을 검사했다. 실제 ESP32 Flash, 장착 각도, 부하 및 전원/토크 해제는 아직 확인하지 않았다.

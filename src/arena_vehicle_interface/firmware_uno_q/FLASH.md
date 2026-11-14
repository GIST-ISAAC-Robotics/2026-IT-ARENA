# 향후 UNO Q MCU 영구 설치·복원 절차

**이번 작업에서 실행한 것은 오프라인 빌드·시험뿐이다. 실제 장치 접근·업로드·냉간부팅 확인은 하지 않았다.** 아래 도구도 `--execute-local` 없이는 OpenOCD/ADB/SSH/포트를 열지 않는다.

## 이미지 형식과 stock loader

이 이미지의 벡터 테이블은 STM32 internal flash `0x08000000`이며 MCUboot/Arduino sketch slot을 사용하지 않는다. native Zephyr kernel 전체다. Arduino stock loader의 ZSK header는 기존 kernel 내부의 sketch 진입점 또는 LLEXT를 처리한다. native kernel에16 B ZSK header를 붙여 기존 `0x08100000` sketch slot에 넣으면 호환된다고 볼 수 없다. 따라서 **STM32 Arduino loader를 교체**하고, 일반 Arduino App Lab/IDE의 다음 업로드는 준비한 native 이미지를 덮어쓸 수 있다. Linux eMMC/OS를 재설치하는 작업은 아니다.

근거:

- [ArduinoCore-zephyr loader/main.c](https://github.com/arduino/ArduinoCore-zephyr/blob/0e2f265190d9fc4166a467215129e1ea29e3f1dd/loader/main.c)
- [ZSK header](https://github.com/arduino/ArduinoCore-zephyr/blob/0e2f265190d9fc4166a467215129e1ea29e3f1dd/cores/arduino/zephyr_sketch_header.h)
- [stock flash_sketch.cfg](https://github.com/arduino/ArduinoCore-zephyr/blob/0e2f265190d9fc4166a467215129e1ea29e3f1dd/variants/arduino_uno_q_stm32u585xx/flash_sketch.cfg)
- [official remoteocd flash.go](https://github.com/arduino/remoteocd/blob/7109bed7d4f138795d7a5c07c0b9d292fd81d77d/flash.go): UNO Q Linux의 `/opt/openocd/bin/openocd`, search `/opt/openocd`, `/opt/openocd/share/openocd/scripts`, `openocd_gpiod.cfg`를 사용한다. 현재 설치 장치에서 이 파일/버전을 아직 확인하지 않았다.

## 실행 전 실물 조건

구동 모터와 서보의 전력 공급을 분리한다. **Jetson↔UNO Q UART는 분리하거나 양쪽 논리 전원이 모두 정상인 상태**여야 한다. 한쪽만 무전원인데 UART 신호를 계속 연결하지 않는다. UNO Q USB를 임시 공급원으로 쓰더라도 VIN과의 전원 우선순위/USB 전류0을 가정하지 않는다. 운영 USB 미연결 조건과 정비 전원을 구별한다.

UNO Q 실물 라벨/보드 모델에서 STM32U585임을 확인한다. 도구는 DBGMCU IDCODE family `0x482`, flash size2048 KiB와96-bit UID를 대조한다. **0x482만으로 U575와 U585를 구별할 수 없으므로 정확한 U585 판정을 IDCODE 하나로 주장하지 않는다.** 보드 모델 확인 인자가 추가로 필요하다. ID/flash size/UID 읽기 실패, 보호 설정으로 백업 불가, OpenOCD 비호환이면 중단하며 unlock/RDP/option byte 변경을 시도하지 않는다.

향후 UNO Q Linux에 배포 묶음을 복사하고, 설치된 `/opt/openocd/openocd_gpiod.cfg`가 해당 UNO Q의 interface/target 구성인지 읽어 확인한다. 다른 upload/reset hook·erase 명령을 넣은 임의 config를 사용하지 않는다. Arduino 업로드 서비스/다른 디버거를 종료하여 MCU 디버그 포트를 한 담당만 소유한다. 아래 경로는 설명용 설치 예시이며 현재 장치에 생성하지 않았다.

```bash
# 기본 dry-run: 파일 해시/벡터 테이블 검증과 계획 출력만.
python3 /home/arduino/arena/bench/flash_native.py \
  --firmware-dir /home/arduino/arena/bench \
  --backup-dir /home/arduino/arena-backups/first-native
```

향후 별도 실물 작업에서 명시 실행하면 다음 순서를 강제한다.

1. reset/halt → family/size 읽기 → 전체2 MiB flash를 서로 독립적으로 두 번 읽기.
2. 길이·SHA256 일치 확인 → `original_flash.bin`, `independent_read.bin`, `backup_manifest.json` 보존. 기존 비어 있지 않은 백업 폴더는 덮어쓰지 않는다.
3. 별도 OpenOCD 세션에서 동일 UID 재확인 → 선택 `zephyr.bin`을 `0x08000000`에 필요한 섹터만 erase/write → verify → reset run.
4. option bytes, mass erase/unlock, Linux eMMC는 쓰지 않는다. 실패 시 원인·로그를 보존하고 자동 재시도하지 않는다.

```bash
# 장비 준비/모델·전원 확인 이후에만 사용. 이번 작업에서는 실행하지 않았다.
sudo python3 /home/arduino/arena/bench/flash_native.py \
  --firmware-dir /home/arduino/arena/bench \
  --backup-dir /home/arduino/arena-backups/first-native \
  --execute-local --confirmed-board ARDUINO_UNO_Q_STM32U585 \
  --ack-motor-servo-power-disconnected --ack-uart-isolated-or-both-powered
```

`--action backup`은 쓰지 않고 전체 백업 후 MCU를 halt 상태에 둔다. CPU 재개는 자동으로 하지 않으므로 후속 작업/전원 절차를 명시한다. flash/restore 실행은 끝에서 reset run하지만 성공 영수증도 냉간부팅 확인을 뜻하지 않는다.

## 원본 복원

같은 MCU의 백업 SHA256/UID가 일치할 때만 원본2 MiB를 복원한다. flash option bytes는 앞 작업에서 변경하지 않았고 복원에서도 쓰지 않는다. option bytes가 외부 도구에 의해 바뀌었다면 이 도구로 원본 상태를 복원했다고 주장할 수 없다.

```bash
# 먼저 dry-run; 실행 플래그/확인은 위와 동일한 실물 조건에서만 덧붙인다.
python3 /home/arduino/arena/bench/flash_native.py --action restore \
  --backup-dir /home/arduino/arena-backups/first-native
```

원본 백업을 잃어버렸다면 stock Arduino loader의 정확한 core 버전과 공식 bootloader 업로드 경로를 따로 확보해야 한다. 임의 이미지나 Linux 전체 재플래시로 복원을 대신하지 않는다. 이후 **전원 재인가 후 MCU 자동 기동, 기본 미무장/PWM0, UART 재접속 무출력, watchdog/명령 만료의 실제 PWM0, encoder·IMU 처리율**을 별도 검증한다. 여기의 빌드·flash verify는 차량 주행/정지나 전원 고장 내성을 증명하지 않는다.

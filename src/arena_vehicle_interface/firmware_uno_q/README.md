# UNO Q native STM32 펌웨어

Jetson의 ROS/보호층과 USB 서보를 유지하며 UNO Q STM32U585가 **USART1 1 Mbps, TIM3 x4 encoder, TIM4 20 kHz PWM/DIR, SPI2 LSM6DSOX FIFO**를 맡는 실제 Zephyr 구현이다. 물리 STOP은 사용자 지시로 포함하지 않는다. `app.overlay`는 TIM3과 ADC/PWM 중복, TIM4 PB9와 IMU CS 중복, USART1 console/logging 중복을 해제한다.

`unconfigured`와 `bench` 이미지를 구분한다. unconfigured는 진단/센서용이며 runtime ARM이 거절된다. bench는 명시 BNV1 ARM/PULSE로 모터만 최대15%·5초 단발 구동할 수 있다. **둘 모두 부팅 기본 PWM0/미무장**이고 자동 실행은 없다. installed 모드는 측정·선택한 방향과 PI/출력/속도 프로필을 별도 JSON으로 제공하면 실제 PI→MD20A 출력 경로를 빌드한다. 현재 설치 프로필은 미완료이며 샘플 게인을 실제 값으로 채우지 않았다.

## 오프라인 빌드

독립 WSL 캐시만 사용한다: `/home/jinhyeong/.cache/it-arena/uno-q-20261114`. 기존 WSL ROS 빌드/설치 디렉터리는 변경하지 않는다.

```bash
REPO='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
bash "$REPO/scripts/build_uno_q_firmware.sh" "$REPO" --bootstrap
bash "$REPO/scripts/build_uno_q_firmware.sh" "$REPO" unconfigured
bash "$REPO/scripts/build_uno_q_firmware.sh" "$REPO" bench
# 설치 프로필은 실측/선택 값이 준비된 뒤 경로를 명시한다.
# bash "$REPO/scripts/build_uno_q_firmware.sh" "$REPO" /absolute/installed-profile.json
```

출력은 `build/uno_q_firmware_20261114/{unconfigured,bench}/`의 ELF/bin/hex/map, DTS/config, profile/header, 빌드 로그와 `firmware_manifest.json`이다. manifest는 공유 portable 헤더를 포함한 소스 해시, 바이너리 해시, toolchain/module/driver commit과 build command를 기록한다. 프로젝트 내부 절대 경로·SDK가 달라지면 ELF 바이트가 달라질 수 있다. manifest는 입력과 버전을 재현하는 근거이며 이 문서만으로 타 환경 bit-for-bit 동일성을 주장하지 않는다.

```bash
python "$REPO/tests/test_uno_q_firmware.py" -v
```

C++ 시험은 합성 입력의 ASan/UBSan 및 `-Os` 빌드다. ESV 재생·세대·기한·분할/CRC, ARM carry, 새 DRIVE/USB ACK 불일치, 벤치 한계/만료, encoder wrap과 실제 PI 출력 경로를 검증한다. 실차 부호·게인·토크·냉간부팅·차체정지 검증은 아니다.

## 제어와 고장 경계

메인 제어는5 ms, encoder 피드백은 기본20 ms다. UART는 IRQ/ring buffer, IMU는 낮은 우선순위 SPI worker이므로 blocking RPC/USB 응답을 제어에서 기다리지 않는다. MCU 명령/서보/host 기한은 MCU 시계에 묶는다. 출력 중 독립1 ms timer guard가 마지막 제어 진행·명령·서보/벤치 기한 중 빠른 기한에 PWM 채널을 끄고 PB8을 LOW로 설정한다. HAL 오류도 LOW를 고정한다.

IWDG200 ms는 정상 메인 제어 진행에서만 feed한다. ISR 또는 별도 타이머가 watchdog을 무조건 갱신하지 않는다. 제어 hang·IRQ 정지 시 리셋이 예상되지만, **리셋은 모터 전력 차단이 아니다**. 리셋/전원순서 동안 실제 pin high-Z·MD20A 입력 상태, 측정 PWM0 시각, 전원/배선 건전성은 후속 실물 확인 사항이다. SPI 성공만으로 IMU를 ready로 만들지 않고 accel·gyro FIFO 관측 각각50 ms 신선도를 요구한다. FIFO overrun/SPI 오류는 IMU 준비를 해제한다.

IMU는 SPI mode3 1 MHz, ±8g/±500dps, accel/gyro208 Hz, FIFO timestamp DEC1, watermark8, 압축 해제다. 기존68 B frame을 유지하고 ESP32 전용 flash/PSRAM 필드에 허구 측정을 채우지 않는다. 큐 손실은 계수/sequence로 노출한다.

프로토콜은 [PROTOCOL.md](PROTOCOL.md), 향후 MCU 백업·영구 업로드·복원은 [FLASH.md](FLASH.md)를 따른다. 현재 기기 접근/업로드는 실행하지 않았다.

## 고정 소스

- [Arduino Zephyr a12e8f3](https://github.com/arduino/zephyr/tree/a12e8f37553a33367745b7df0cb064215a4fa7bf): `boards/arduino/uno_q`, STM32 QDEC/PWM/UART/WDT 드라이버.
- [ST LSM6DSOX driver 2e86793](https://github.com/STMicroelectronics/lsm6dsox-pid/tree/2e86793125f7f46b6195fce93a2b3cb76a51e564).
- [Zephyr SDK1.0.1](https://github.com/zephyrproject-rtos/sdk-ng/releases/tag/v1.0.1): official checksum으로 minimal/ARM archive 검증.
- [ST STM32U585 DS13086](https://www.st.com/resource/en/datasheet/stm32u585ai.pdf): PA6/PA7 TIM3, PB8 TIM4_CH3 AF2.

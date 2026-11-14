#!/usr/bin/env bash
# 독립 WSL 캐시에서만 의존성을 준비한다. 장치 열기/flash 명령은 없다.
set -euo pipefail
UNO_CACHE=/home/jinhyeong/.cache/it-arena/uno-q-20261114
UNO_REPO="${1:?repository Linux path required}"
UNO_MODE="${2:---bootstrap}"
if [[ "$UNO_MODE" != --bootstrap ]]; then
  test -x "$UNO_CACHE/venv/bin/python" && test -x "$UNO_CACHE/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc" || {
    printf '%s\n' 'Run --bootstrap once before offline builds.' >&2; exit 2;
  }
  export PATH="$UNO_CACHE/venv/bin:/usr/bin:/bin"
  exec "$UNO_CACHE/venv/bin/python" "$UNO_REPO/src/arena_vehicle_interface/firmware_uno_q/build.py" --repository "$UNO_REPO" --cache "$UNO_CACHE" --profile "$UNO_MODE"
fi
mkdir -p "$UNO_CACHE/downloads"
cd "$UNO_CACHE"
if [[ ! -x venv/bin/python ]]; then python3 -m venv --without-pip venv; fi
export PIP_CACHE_DIR="$UNO_CACHE/pip-cache"
if ! venv/bin/python -m pip --version >/dev/null 2>&1; then
  curl -fL --retry 3 https://bootstrap.pypa.io/get-pip.py -o downloads/get-pip.py
  venv/bin/python downloads/get-pip.py 'pip==25.3' 'setuptools==80.9.0' 'wheel==0.45.1'
fi
venv/bin/python -m pip -q install 'west==1.5.0' 'ninja==1.13.0' 'cmake==3.31.10'
export PATH="$UNO_CACHE/venv/bin:$PATH"
if [[ ! -d zephyr/.git ]]; then
  git clone --depth 1 --branch zephyr-arduino-v4.4.2 https://github.com/arduino/zephyr.git zephyr
fi
test "$(git -C zephyr rev-parse HEAD)" = a12e8f37553a33367745b7df0cb064215a4fa7bf
if [[ ! -d .west ]]; then west init -l zephyr; fi
west update cmsis cmsis_6 hal_stm32
venv/bin/python -m pip -q install -r zephyr/scripts/requirements-base.txt
UNO_SDK_VERSION=1.0.1
UNO_RELEASE="https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v$UNO_SDK_VERSION"
for UNO_ARCHIVE in "zephyr-sdk-${UNO_SDK_VERSION}_linux-x86_64_minimal.tar.xz" toolchain_gnu_linux-x86_64_arm-zephyr-eabi.tar.xz; do
  if [[ ! -f "downloads/$UNO_ARCHIVE" ]]; then curl -fL --retry 3 "$UNO_RELEASE/$UNO_ARCHIVE" -o "downloads/$UNO_ARCHIVE"; fi
done
if [[ ! -f downloads/sdk-sha256.sum ]]; then curl -fL --retry 3 "$UNO_RELEASE/sha256.sum" -o downloads/sdk-sha256.sum; fi
cd downloads
grep -E ' (zephyr-sdk-1.0.1_linux-x86_64_minimal.tar.xz|toolchain_gnu_linux-x86_64_arm-zephyr-eabi.tar.xz)$' sdk-sha256.sum | sha256sum -c -
cd "$UNO_CACHE"
if [[ ! -d zephyr-sdk-1.0.1 ]]; then tar -xf downloads/zephyr-sdk-1.0.1_linux-x86_64_minimal.tar.xz; fi
if [[ ! -x zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc ]]; then
  mkdir -p zephyr-sdk-1.0.1/gnu
  tar -xf downloads/toolchain_gnu_linux-x86_64_arm-zephyr-eabi.tar.xz -C zephyr-sdk-1.0.1/gnu
fi
export ZEPHYR_SDK_INSTALL_DIR="$UNO_CACHE/zephyr-sdk-1.0.1"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
if [[ "$UNO_MODE" = --bootstrap ]]; then
  printf '%s\n' 'UNO Q isolated toolchain bootstrap complete.'
  exit 0
fi
exec venv/bin/python "$UNO_REPO/src/arena_vehicle_interface/firmware_uno_q/build.py" --repository "$UNO_REPO" --cache "$UNO_CACHE" --profile "$UNO_MODE"

"""보존 SDK와 고정 upstream 소스의 시각 산출 경로를 기록한다. 장치 미접속."""
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
COMMIT = '99478e5fb90de3b4a6db0080acacd373f8b36869'
UPSTREAM = ROOT/'build/c1_timestamp_source_r127'


def main():
    files = {
        'sdk/src/dataunpacker/unpacker/handler_normalnode.cpp': UPSTREAM/'handler_normalnode.cpp',
        'sdk/src/dataunpacker/dataunpacker.cpp': UPSTREAM/'dataunpacker.cpp',
        'sdk/src/arch/linux/timer.cpp': UPSTREAM/'timer.cpp',
        'sdk/include/sl_lidar_cmd.h': UPSTREAM/'sl_lidar_cmd.h',
        'sdk/src/sl_lidar_driver.cpp': ROOT/'build/c1_hardware_2026_11_10/remote/official-current-sdk-driver.cpp',
    }
    texts = {name: path.read_text(encoding='utf8') for name, path in files.items()}
    driver = texts['sdk/src/sl_lidar_driver.cpp']
    handler = texts['sdk/src/dataunpacker/unpacker/handler_normalnode.cpp']
    assert '_timing_desc.native_timestamp_support = false' in driver
    assert '_scan_begin_timestamp_uS[operationBufID] = currentSampleTsUs' in driver
    assert 'getCurrentTimestamp_uS() - _getSampleDelayOffsetInLegacyMode' in handler
    assert 'return getus();' in texts['sdk/src/dataunpacker/dataunpacker.cpp']
    assert 'CLOCK_MONOTONIC' in texts['sdk/src/arch/linux/timer.cpp']
    recorded = json.loads((ROOT/'build/c1_hardware_2026_11_10/remote/official-current-sdk-source.json').read_bytes())
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    assert recorded['commit'] == COMMIT
    assert recorded['driver_sha256'] == sha(files['sdk/src/sl_lidar_driver.cpp'])
    # Packed normal node = uint8 + uint16 + uint16; C-series native UART=460800.
    transmission_us = 1_000_000 * 5 * 10 // 460800
    offset_us = 200 + (200 >> 1) + transmission_us + 0
    result = dict(
        commit=COMMIT,
        sources={name: dict(sha256=sha(path), local=path.relative_to(ROOT).as_posix(),
                           url=f'https://github.com/Slamtec/rplidar_sdk/blob/{COMMIT}/{name}')
                 for name, path in files.items()},
        answer_type='0x81', nominal_sample_us=200, node_bytes=5, uart_baud=460800,
        transmission_estimate_us=transmission_us, total_offset_us=offset_us,
        sdk_timestamp='host_monotonic_at_decode_minus_constant_offset',
        batch_timestamp='timestamp_of_sync_node_starting_completed_revolution',
        per_node_device_timestamp_in_normal_payload=False,
        usb_and_scheduling_delay_individually_measured=False,
        physical_acquisition_timing_verified=False, calibration_applied=False,
        scope='source audit of recorded SDK commit, not physical timing measurement',
    )
    with (A/'report.json').open('x', encoding='utf8') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps({k: v for k, v in result.items() if k != 'sources'}))


if __name__ == '__main__':
    main()

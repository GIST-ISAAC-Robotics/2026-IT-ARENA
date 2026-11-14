#!/usr/bin/env python3
"""장치 I/O 없이 동결 소스→독립 Zephyr build→ELF/bin/manifest를 만든다."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request

ZEPHYR_COMMIT = 'a12e8f37553a33367745b7df0cb064215a4fa7bf'
ST_COMMIT = '2e86793125f7f46b6195fce93a2b3cb76a51e564'
PREFIX = Path('src/arena_vehicle_interface/firmware_uno_q')
PINNED_REPOS = {'zephyr': ZEPHYR_COMMIT,
                'modules/hal/cmsis': '512cc7e895e8491696b61f7ba8066b4a182569b8',
                'modules/hal/cmsis_6': '30a859f44ef8ab4dc8f84b03ed586fd16ccf9d74',
                'modules/hal/stm32': 'fc11896dd39cfca37bf9b4aeaaa2df8861b81875'}

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def profile_fingerprint(profile):
    canonical = json.dumps(profile, sort_keys=True, separators=(',', ':'), ensure_ascii=True, allow_nan=False)
    return hashlib.sha256(canonical.encode('ascii')).hexdigest()

def profile_header(profile):
    required = {'schema', 'mode', 'assembly_id', 'encoder_direction', 'motor_direction_confirmed',
                'forward_dir_high', 'maximum_duty', 'kp', 'ki', 'maximum_speed_mps',
                'maximum_steering_rad', 'feedback_period_us'}
    if set(profile) != required or profile['schema'] != 'uno_q_profile_v1':
        raise ValueError('exact uno_q_profile_v1 schema required')
    if profile['mode'] not in ('unconfigured', 'bench', 'installed'):
        raise ValueError('unknown profile mode')
    if type(profile['assembly_id']) is not str or not profile['assembly_id'].strip():
        raise ValueError('nonempty assembly_id required')
    for key in ('maximum_duty', 'kp', 'ki', 'maximum_speed_mps', 'maximum_steering_rad'):
        if type(profile[key]) not in (int, float) or not math.isfinite(profile[key]):
            raise ValueError('finite numeric '+key+' required')
    for key in ('forward_dir_high', 'motor_direction_confirmed'):
        if type(profile[key]) is not bool:
            raise ValueError('explicit boolean '+key+' required')
    if type(profile['encoder_direction']) is not int or profile['encoder_direction'] not in (-1, 0, 1):
        raise ValueError('encoder_direction must be -1, 0 or +1')
    if type(profile['feedback_period_us']) is not int or not 5000 <= profile['feedback_period_us'] < 50000:
        raise ValueError('feedback period outside receiver freshness')
    if not 0 < profile['maximum_steering_rad'] <= 0.28475436568260193:
        raise ValueError('steering command outside current selected limit')
    if profile['mode'] == 'installed':
        if not (profile['motor_direction_confirmed'] and profile['encoder_direction'] in (-1, 1)
                and 0 < profile['maximum_duty'] <= 1 and profile['kp'] > 0 and profile['ki'] > 0
                and 0 < profile['maximum_speed_mps'] <= 13/3.6):
            raise ValueError('installed mode requires explicit measured/selected motor profile')
    lines = ['#pragma once', '#include "profile.hpp"',
             'inline constexpr char selected_profile_sha256[]="'+profile_fingerprint(profile)+'";',
             'inline unoq::Profile selected_profile() {',
             '  unoq::Profile p;', '  p.mode=unoq::Mode::'+profile['mode'].capitalize()+';']
    for key in sorted(required - {'schema', 'mode', 'assembly_id'}):
        value = profile[key]
        literal = ('true' if value else 'false') if type(value) is bool else repr(value)
        lines.append('  p.'+key+'='+literal+';')
    return '\n'.join(lines+['  return p;', '}', ''])

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repository', type=Path, required=True)
    ap.add_argument('--cache', type=Path, required=True)
    ap.add_argument('--profile', default='unconfigured')
    args = ap.parse_args()
    root, cache = args.repository.resolve(), args.cache.resolve()
    if cache != Path('/home/jinhyeong/.cache/it-arena/uno-q-20261114'):
        raise ValueError('isolated cache path required')
    for repository, expected in PINNED_REPOS.items():
        actual = subprocess.check_output(['git','-C',str(cache/repository),'rev-parse','HEAD'],text=True).strip()
        if actual != expected:
            raise ValueError('pinned repository mismatch: '+repository)
        subprocess.run(['git','-C',str(cache/repository),'diff','--exit-code','HEAD','--'],check=True,stdout=subprocess.DEVNULL)
    profile_path = root/PREFIX/'profiles'/(args.profile+'.json')
    if not profile_path.exists():
        profile_path = Path(args.profile).resolve(strict=True)
    profile = json.loads(profile_path.read_text())
    header = profile_header(profile)
    name = profile['mode']
    stage = cache/'snapshot'/name
    files = list((root/PREFIX).rglob('*')) + list((root/'src/arena_vehicle_interface/portable').glob('*.hpp'))
    files += [root/'src/arena_vehicle_interface/firmware_runtime_preview/main/runtime_tx.hpp',
              root/'scripts/build_uno_q_firmware.sh', root/'docs/hardware/schematics/uno_q_v1.json']
    files += list((root/'tests').glob('test_uno_q_firmware*'))
    files = [p for p in files if p.is_file() and '__pycache__' not in p.parts]
    hashes = {p.relative_to(root).as_posix(): sha(p) for p in files}
    for p in files:
        dst = stage/p.relative_to(root)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(p, dst)
    generated = stage/'selected_profile.hpp'
    generated.write_text(header)
    driver = cache/'third_party'/'lsm6dsox-pid'/ST_COMMIT
    driver.mkdir(parents=True, exist_ok=True)
    for filename in ('lsm6dsox_reg.c', 'lsm6dsox_reg.h', 'LICENSE'):
        dest = driver/filename
        if not dest.exists():
            with urllib.request.urlopen('https://raw.githubusercontent.com/STMicroelectronics/lsm6dsox-pid/'+ST_COMMIT+'/'+filename) as r:
                dest.write_bytes(r.read())
    env = dict(os.environ, ZEPHYR_BASE=str(cache/'zephyr'),
               ZEPHYR_SDK_INSTALL_DIR=str(cache/'zephyr-sdk-1.0.1'), ZEPHYR_TOOLCHAIN_VARIANT='zephyr')
    build = cache/'build'/name
    command = ['west', 'build', '-b', 'arduino_uno_q/stm32u585xx', '-d', str(build),
               str(stage/PREFIX), '--', '-DARENA_PROFILE_HEADER='+str(generated),
               '-DST_DRIVER_DIR='+str(driver), '-DUSER_CACHE_DIR='+str(cache/'zephyr-cache')]
    result = subprocess.run(command, cwd=cache, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    build.mkdir(parents=True, exist_ok=True)
    (build/'build_stdout.log').write_text(result.stdout)
    print('\n'.join(result.stdout.splitlines()[-65:]))
    result.check_returncode()
    for rel, before in hashes.items():
        if sha(root/rel) != before:
            raise RuntimeError('source changed during build: '+rel)
    output = root/'build/uno_q_firmware_20261114'/name
    output.mkdir(parents=True, exist_ok=True)
    for filename in ('zephyr.elf', 'zephyr.bin', 'zephyr.hex', 'zephyr.map', '.config', 'zephyr.dts'):
        shutil.copyfile(build/'zephyr'/filename, output/filename)
    shutil.copyfile(build/'build_stdout.log', output/'build.log')
    shutil.copyfile(generated, output/'selected_profile.hpp')
    for filename in ('README.md', 'PROTOCOL.md', 'FLASH.md', 'flash_native.py'):
        shutil.copyfile(stage/PREFIX/filename, output/filename)
    (output/'profile.json').write_text(json.dumps(profile, ensure_ascii=False, indent=2)+'\n')
    (output/'requirements-lock.txt').write_text(subprocess.check_output([sys.executable, '-m', 'pip', 'freeze'], text=True))
    toolchain = cache/'zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc'
    compiler_version = subprocess.check_output([toolchain, '--version'], text=True).splitlines()[0]
    artifacts = {p.name: sha(p) for p in output.iterdir() if p.is_file() and p.name != 'firmware_manifest.json'}
    manifest = dict(schema='uno_q_firmware_build_v1', mode=name, source_files=hashes,
                    artifacts=artifacts, zephyr_commit=ZEPHYR_COMMIT,
                    sdk='1.0.1', compiler=compiler_version, st_driver_commit=ST_COMMIT,
                    st_driver_sha256={p.name:sha(p) for p in driver.iterdir() if p.is_file()},
                    build_command=command, flash_address='0x08000000',
                    image_kind='standalone_native_zephyr_replaces_arduino_mcu_loader',
                    mcuboot=False, persistent_storage='STM32 internal flash',
                    hardware_accessed=False, firmware_uploaded=False, cold_boot_verified=False,
                    output_by_default=False, selected_profile=profile,
                    profile_sha256=profile_fingerprint(profile),
                    module_commits={name:subprocess.check_output(['git','-C',str(cache/path),'rev-parse','HEAD'],text=True).strip()
                                    for name,path in {'cmsis':'modules/hal/cmsis','cmsis_6':'modules/hal/cmsis_6','hal_stm32':'modules/hal/stm32'}.items()},
                    sdk_archive_sha256={p.name:sha(p) for p in (cache/'downloads').glob('*.tar.xz')})
    (output/'firmware_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    print(output)

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""UNO Q MCU 전용 향후 작업 도구. 기본은 파일 검증·계획 출력뿐이며 장치를 열지 않는다."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

SIZE = 2 * 1024 * 1024
CONFIRM = 'ARDUINO_UNO_Q_STM32U585'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def tcl_path(path):
    value = str(path.resolve()).replace('\\', '/')
    if any(c in value for c in '{}\r\n'):
        raise ValueError('Tcl path contains forbidden delimiter')
    return '{'+value+'}'

def identity_tcl(expected_uid=None):
    text = '''init
reset halt
set uno_id [lindex [read_memory 0xE0044000 32 1] 0]
set uno_size [lindex [read_memory 0x0BFA07A0 16 1] 0]
set uno_uid_words [read_memory 0x0BFA0700 32 3]
set uno_uid [format "%08x%08x%08x" [lindex $uno_uid_words 0] [lindex $uno_uid_words 1] [lindex $uno_uid_words 2]]
if {($uno_id & 0xfff) != 0x482} {error "not STM32U575/U585 device family"}
if {$uno_size != 2048} {error "not expected 2 MiB flash"}
echo [format "UNO_Q_ID=%08x UID=%s SIZE=%d" $uno_id $uno_uid $uno_size]
'''
    if expected_uid is not None:
        if not re.fullmatch('[0-9a-f]{24}', expected_uid):
            raise ValueError('invalid backup UID')
        text += 'if {$uno_uid ne "'+expected_uid+'"} {error "backup belongs to another MCU"}\n'
    return text

def run_openocd(args, script, name):
    cfg = args.backup_dir/(name+'.cfg')
    cfg.write_text(script, encoding='ascii')
    command = [args.openocd, '-s', '/opt/openocd', '-s', '/opt/openocd/share/openocd/scripts',
               '-c', 'gdb_port disabled', '-c', 'tcl_port disabled',
               '-c', 'telnet_port disabled', '-f', str(args.board_config.resolve()), '-f', str(cfg)]
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=180)
    (args.backup_dir/(name+'.log')).write_text(result.stdout)
    result.check_returncode()
    match = re.search(r'UNO_Q_ID=([0-9a-f]{8}) UID=([0-9a-f]{24}) SIZE=(\d+)', result.stdout)
    if not match:
        raise RuntimeError('no verified identity marker; do not proceed')
    return dict(device_id=match[1], uid=match[2], flash_kib=int(match[3]))

def verified_backup(directory):
    record = json.loads((directory/'backup_manifest.json').read_text())
    image = directory/'original_flash.bin'
    if record.get('board_confirmation') != CONFIRM or record.get('flash_kib') != 2048:
        raise ValueError('backup identity not confirmed')
    if image.stat().st_size != SIZE or sha(image) != record.get('sha256'):
        raise ValueError('original full flash backup missing/corrupt')
    if record.get('independent_read_sha256') != record['sha256']:
        raise ValueError('second full read did not verify backup')
    return image, record

def verify_image(directory):
    manifest = json.loads((directory/'firmware_manifest.json').read_text())
    if manifest.get('flash_address') != '0x08000000' or manifest.get('mcuboot') is not False:
        raise ValueError('standalone native image manifest required')
    image = directory/'zephyr.bin'
    if not 8 < image.stat().st_size <= SIZE or sha(image) != manifest['artifacts'].get('zephyr.bin'):
        raise ValueError('firmware image hash/size mismatch')
    data = image.read_bytes()
    sp, entry = int.from_bytes(data[:4], 'little'), int.from_bytes(data[4:8], 'little')
    if not (0x20000000 <= sp <= 0x200C0000 and 0x08000001 <= entry < 0x08200000 and entry & 1):
        raise ValueError('vector table does not target expected SRAM/internal flash')
    return image

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--action', choices=['backup','flash','restore'], default='flash')
    ap.add_argument('--firmware-dir', type=Path)
    ap.add_argument('--backup-dir', type=Path, required=True)
    ap.add_argument('--board-config', type=Path, default=Path('/opt/openocd/openocd_gpiod.cfg'),
                    help='reviewed installed Arduino UNO Q OpenOCD interface+target config, no upload hooks')
    ap.add_argument('--openocd', default='/opt/openocd/bin/openocd')
    ap.add_argument('--execute-local', action='store_true')
    ap.add_argument('--confirmed-board', choices=[CONFIRM])
    ap.add_argument('--ack-motor-servo-power-disconnected', action='store_true')
    ap.add_argument('--ack-uart-isolated-or-both-powered', action='store_true')
    args = ap.parse_args(argv)
    image = None
    if args.action == 'flash':
        if args.firmware_dir is None:
            ap.error('--firmware-dir required for flash')
        image = verify_image(args.firmware_dir)
    print(json.dumps(dict(dry_run=not args.execute_local, action=args.action, image=str(image),
                          flash_address='0x08000000', backup_bytes=SIZE,
                          board_config=str(args.board_config), backup_dir=str(args.backup_dir),
                          option_byte_writes=False, linux_storage_writes=False), indent=2))
    if not args.execute_local:
        return 0  # no subprocess, no device open, no backup-directory creation
    if not (args.confirmed_board == CONFIRM and args.ack_motor_servo_power_disconnected
            and args.ack_uart_isolated_or_both_powered):
        ap.error('execution requires board identity and both isolation acknowledgements')
    if not args.board_config.is_file():
        ap.error('reviewed local UNO Q OpenOCD configuration must exist')
    if args.action in ('backup','flash'):
        if args.backup_dir.exists() and any(args.backup_dir.iterdir()):
            ap.error('use a new empty backup directory; originals are never overwritten')
        args.backup_dir.mkdir(parents=True, exist_ok=True)
        original, second = args.backup_dir/'original_flash.bin', args.backup_dir/'independent_read.bin'
        script = identity_tcl() + 'dump_image '+tcl_path(original)+' 0x08000000 0x200000\n'
        script += 'dump_image '+tcl_path(second)+' 0x08000000 0x200000\nshutdown\n'
        identity = run_openocd(args, script, 'backup_read')
        if original.stat().st_size != SIZE or second.stat().st_size != SIZE or sha(original) != sha(second):
            raise RuntimeError('independent full flash reads disagree; no write permitted')
        record = dict(identity, sha256=sha(original), independent_read_sha256=sha(second),
                      board_confirmation=CONFIRM, device_id_scope='U575/U585 family; exact U585 confirmed from board label',
                      board_config_sha256=sha(args.board_config), option_bytes_changed=False)
        (args.backup_dir/'backup_manifest.json').write_text(json.dumps(record, indent=2)+'\n')
        if args.action == 'backup':
            return 0  # target remains halted; deliberate recovery/resume is required
    original, record = verified_backup(args.backup_dir)
    if args.action == 'restore':
        image = original
    script = identity_tcl(record['uid'])
    script += 'flash write_image erase '+tcl_path(image)+' 0x08000000 bin\n'
    script += 'verify_image '+tcl_path(image)+' 0x08000000 bin\nreset run\nshutdown\n'
    result = run_openocd(args, script, args.action+'_verify')
    (args.backup_dir/(args.action+'_receipt.json')).write_text(json.dumps(dict(result, image_sha256=sha(image), verified=True,
                       cold_boot_verified=False, option_bytes_changed=False), indent=2)+'\n')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())

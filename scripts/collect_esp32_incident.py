#!/usr/bin/env python3
"""보존된 로컬 파일만으로 ESP32 사건 증거 묶음을 만든다. 장치 접근 없음."""
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'artifacts/validation/2026-11-15/esp32_incident_v1'
LOCAL = ROOT / 'build/esp32_incident_20261115'


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8', newline='\n')


def main():
    # A fresh folder prevents replacing evidence from a previous collection.
    OUT.mkdir(parents=True, exist_ok=False)
    LOCAL.mkdir(parents=True, exist_ok=False)
    (OUT / 'evidence').mkdir()
    sources = []
    for folder in ('mounted_motor_start15_5s_20261114', 'mounted_motor_guarded_5s_20261114'):
        sources.extend(sorted((ROOT / 'build' / folder).iterdir()))
    records = []
    for source in sources:
        if not source.is_file():
            continue
        relative = source.relative_to(ROOT).as_posix()
        public = source.suffix == '.json'
        destination = OUT / 'evidence' / source.name if public else None
        if destination:
            shutil.copyfile(source, destination)
            assert sha(destination) == sha(source)
        records.append({'source': relative, 'bytes': source.stat().st_size, 'sha256': sha(source),
                        'public_copy': destination.relative_to(OUT).as_posix() if destination else None})
    byname = {p.name: p for p in sources if p.is_file()}
    commands = [json.loads(line) for line in byname['start15-reconnect-commands.jsonl'].read_text().splitlines()]
    assert [x['wire'] for x in commands] == ['?\n', 'X\n']
    write(OUT / 'reconnect_command_excerpt.json', {
        'source_sha256': sha(byname['start15-reconnect-commands.jsonl']),
        'scope': 'two records in the locally recovered reconnect command file only',
        'records': commands, 'contains_arm_or_pulse': False,
        'query_to_stop_seconds': (commands[1]['host_ns']-commands[0]['host_ns'])/1e9})
    pulses = []
    for name, supply in [('powered-5s.json', True), ('powered-start-75.json', True), ('start15-no-power-guard.json', False)]:
        value = json.loads(byname[name].read_text())
        end = value['output_end']
        pulses.append({'source': 'evidence/' + name, 'motor_supply_connected_operator_report': supply,
                       'duty_permille': value['duty_permille'], 'requested_duration_ms': value['duration_ms'],
                       'logical_output_duration_ms': (end['ended_us']-end['started_us'])/1000,
                       'end_reason': end['reason'], 'end_duty_permille': end['duty_permille'],
                       'end_pwm_pin': end['pwm_pin'], 'end_armed': end['armed'],
                       'encoder_delta': value['encoder_delta']['pcnt_count']})
    assert all(p['end_duty_permille'] == 0 and p['end_pwm_pin'] == 0 and not p['end_armed'] for p in pulses)
    manual = json.loads(byname['manual-encoder-result.json'].read_text())
    correction = json.loads(byname['operator-direction-correction.json'].read_text())
    assert manual['delta']['pcnt_count'] == manual['delta']['software_count'] == 690
    assert correction['corrected_direction'] == 'backward'
    write(OUT / 'observations.json', {'pulses': pulses,
          'manual_encoder_delta': 690, 'corrected_manual_direction': 'backward',
          'manual_pwm_zero': manual['pwm_zero'], 'manual_unarmed': manual['unarmed'],
          'reconnect_stop_confirmed': False,
          'wall_clock_timeline_source': 'docs/hardware/MOUNTED_DC_TEST_2026_11_14.md; full original UART not available locally',
          'electrical_failure_time_confirmed': False, 'electrical_cause_confirmed': False})
    rom = byname['esp32-rom-query.log'].read_text()
    public_rom = '\n'.join('Serial port <ESP32_USB_SERIAL_PORT>:' if line.startswith('Serial port ') else line
                           for line in rom.splitlines()) + '\n'
    (OUT / 'rom_query_excerpt.txt').write_text(public_rom, encoding='utf-8', newline='\n')
    manifest = {'scope': 'local files recovered during 2026-11-14 tests, collected 2026-11-15',
                'hardware_access': False, 'original_files_modified': False,
                'missing_local_evidence': ['complete start15-run UART/command stream', 'kernel USB disconnect journal',
                                           'power/current/temperature waveform', 'stable isolated-board resistance measurement'],
                'files': records}
    write(OUT / 'manifest.json', manifest)
    archive = LOCAL / 'esp32_incident_local_evidence.zip'
    with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED) as z:
        for item in records:
            z.write(ROOT / item['source'], item['source'])
        z.writestr('manifest.json', json.dumps(manifest, ensure_ascii=False, indent=2))
    write(OUT / 'local_bundle.json', {'path': archive.relative_to(ROOT).as_posix(), 'bytes': archive.stat().st_size,
                                     'sha256': sha(archive), 'publicly_committed': False})
    print(json.dumps({'sources': len(records), 'public_original_json_copies': sum(x['public_copy'] is not None for x in records),
                      'local_archive_sha256': sha(archive), 'hardware_access': False}))


if __name__ == '__main__':
    main()
